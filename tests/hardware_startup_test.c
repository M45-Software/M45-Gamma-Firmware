#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

typedef int esp_err_t;
typedef unsigned TickType_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_TIMEOUT 0x107
#define M45_FAULT_MESSAGE_MAX 1023
#define ASIC_POWER_OFF_MIN_MS 1000
/* RX_DRAIN_TIMEOUT_UNDER_TEST */
#define ASIC_TEMP_STARTUP_GRACE_MS 3000
#define DOMAIN_REBOOT_STARTUP_GRACE_MS 60000
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(x) (x)
#define pdTICKS_TO_MS(x) (x)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define ESP_RETURN_ON_ERROR(call, ...) do { esp_err_t e_ = (call); if (e_ != ESP_OK) return e_; } while (0)

typedef struct {
    struct { bool hardware_fault; unsigned power_fault; char hardware_fault_msg[1024]; } SYSTEM_MODULE;
    struct { void *active_jobs; } ASIC_TASK_MODULE;
    void *valid_jobs;
    bool ASIC_initalized;
    struct { float actual_frequency, frequency_value, power, current, core_voltage, chip_temp_avg; } POWER_MANAGEMENT_MODULE;
} GlobalState;
typedef struct { bool overclock_enabled, asic_voltage_temp_compensation_enabled; unsigned safety_asic_temp_max_c; } m45_config_t;
static const m45_config_t config = {.safety_asic_temp_max_c = 69};
static TickType_t tick, off_tick, g_asic_temp_grace_until, g_domain_reboot_grace_until;
static int64_t g_asic_temp_unavailable_since_us;
static bool g_regulator_enabled, g_tps546_ready, off_recorded;
static bool work_paused;
static uint16_t g_commanded_voltage_mv;
static uint8_t g_chip_count;
static char status[64];
static unsigned records, shutdowns, attempts, serial_inits, detections, baud_changes, flushes;
static unsigned tps_inits, reset_pulses;
static unsigned pauses, resumes, locks;
static unsigned validation_calls, safety_validation;
static TickType_t idle_wait_ticks;
static float measured_temp;
typedef struct { unsigned valid; } TPS546_StatusSnapshot;
static char last_record[1024];
static char first_record[1024];
enum failure { NONE, NO_CHIP, SERIAL, BAUD, TPS_INIT, RESET, FAN_INIT, FAN_START, TPS_SET, MONITOR };
static enum failure failing;
static int failures_left;

static bool fail(enum failure kind) {
    if (failing != kind || failures_left == 0) return false;
    if (failures_left > 0) --failures_left;
    return true;
}
static size_t strlcpy(char *dst, const char *src, size_t n) { snprintf(dst, n, "%s", src); return strlen(src); }
static const char *esp_err_to_name(esp_err_t e) { return e == ESP_OK ? "ESP_OK" : "ESP_FAIL"; }
static TickType_t xTaskGetTickCount(void) { return tick; }
static void vTaskDelay(TickType_t duration) { tick += duration; }
static void set_hw_status(const char *text) { strlcpy(status, text, sizeof(status)); }
static void bitaxe_gamma602_clear_jobs(GlobalState *s) { (void)s; }
static void format_fault_context(GlobalState *s, const char *r, char *dst, size_t n) {
    char original[1024]; strlcpy(original, r, sizeof(original));
    snprintf(dst, n, "%s; actual %.0f MHz", original, s->POWER_MANAGEMENT_MODULE.actual_frequency);
}
static esp_err_t m45_fault_log_record(const char *reason) {
    ++records; strlcpy(last_record, reason, sizeof(last_record));
    if (records == 1) strlcpy(first_record, reason, sizeof(first_record));
    return ESP_OK;
}
static void record_asic_power_off(void) { off_tick = tick; off_recorded = true; }
static bool consume_asic_power_off_tick(TickType_t *out) {
    bool had = off_recorded; *out = off_tick; off_recorded = false; return had;
}
static esp_err_t shutdown_regulator(GlobalState *s, const char *reason) {
    (void)reason;
    ++shutdowns; g_regulator_enabled = false; g_commanded_voltage_mv = 0; g_chip_count = 0;
    s->POWER_MANAGEMENT_MODULE.actual_frequency = 0;
    set_hw_status("regulator shutdown"); record_asic_power_off(); return ESP_OK;
}
static esp_err_t asic_reset_level(int level) {
    assert(level == 0); ++attempts;
    return fail(RESET) ? ESP_FAIL : ESP_OK;
}
static esp_err_t asic_reset_pulse(void) { ++reset_pulses; tick += 200; return ESP_OK; }
static esp_err_t bitaxe_gamma602_start_fan(GlobalState *s) { (void)s; return fail(FAN_INIT) ? ESP_FAIL : ESP_OK; }
static esp_err_t bitaxe_fan_start_for_asic(GlobalState *s) { (void)s; return fail(FAN_START) ? ESP_FAIL : ESP_OK; }
static esp_err_t bitaxe_fan_stop_for_asic(GlobalState *s) { (void)s; return ESP_OK; }
static void clear_power_snapshot(void) {}
static void reset_power_average_window(void) {}
static int gamma_tps546_config(void) { return 0; }
static esp_err_t TPS546_init(int cfg) { (void)cfg; ++tps_inits; tick += 10; return fail(TPS_INIT) ? ESP_FAIL : ESP_OK; }
static const m45_config_t *m45_config_get(void) { return &config; }
static bool read_asic_temp_for_voltage(GlobalState *s, float *temp) { (void)s; *temp = measured_temp; return measured_temp > 0 && measured_temp != 127; }
static bool asic_temp_is_no_reading(float temp) { return temp == 127; }
static void safety_shutdownf(GlobalState *s, const char *fmt, ...) {
    va_list args; va_start(args, fmt);
    vsnprintf(s->SYSTEM_MODULE.hardware_fault_msg, sizeof(s->SYSTEM_MODULE.hardware_fault_msg), fmt, args);
    va_end(args);
    s->SYSTEM_MODULE.hardware_fault = true; s->SYSTEM_MODULE.power_fault = 1;
    s->ASIC_initalized = false; shutdown_regulator(s, s->SYSTEM_MODULE.hardware_fault_msg);
}
static esp_err_t TPS546_snapshot_status(TPS546_StatusSnapshot *s) { s->valid = 1; return ESP_OK; }
static esp_err_t fail_regulator_safety(GlobalState *s, const TPS546_StatusSnapshot *p, const char *msg) {
    (void)p; safety_shutdownf(s, "%s", msg); return ESP_FAIL;
}
static uint16_t m45_config_effective_asic_voltage_mv_for_temp(const m45_config_t *c, float temp) { (void)c; (void)temp; return 1150; }
static int16_t m45_config_asic_voltage_temp_compensation_mv(const m45_config_t *c, float temp) { (void)c; (void)temp; return 0; }
static uint16_t m45_config_effective_asic_frequency_mhz(const m45_config_t *c) { (void)c; return 525; }
static esp_err_t TPS546_set_vout(float volts) {
    if (volts > 0 && off_recorded) assert(tick - off_tick >= ASIC_POWER_OFF_MIN_MS);
    return fail(TPS_SET) ? ESP_FAIL : ESP_OK;
}
static esp_err_t wait_for_regulator_after_enable(GlobalState *s, float volts) {
    (void)volts; ++validation_calls;
    if (safety_validation == validation_calls) {
        s->SYSTEM_MODULE.hardware_fault = true; s->SYSTEM_MODULE.power_fault = 1;
        strlcpy(s->SYSTEM_MODULE.hardware_fault_msg, "TPS safety fault", sizeof(s->SYSTEM_MODULE.hardware_fault_msg));
        s->ASIC_initalized = false; shutdown_regulator(s, "TPS safety fault");
        return ESP_FAIL;
    }
    return ESP_OK;
}
static esp_err_t start_power_monitor(GlobalState *s) { (void)s; return fail(MONITOR) ? ESP_FAIL : ESP_OK; }
static esp_err_t SERIAL_init(void) { ++serial_inits; return fail(SERIAL) ? ESP_FAIL : ESP_OK; }
static void SERIAL_clear_buffer(void) { ++flushes; }
static uint8_t BM1370_init(GlobalState *s) {
    ++detections;
    assert(flushes >= detections);
    const bool no_chip = fail(NO_CHIP);
    tick += no_chip ? 5000 : 1000; // The driver owns bounded read-only retries.
    if (no_chip) return 0;
    s->POWER_MANAGEMENT_MODULE.actual_frequency = 525; return 1;
}
static int BM1370_set_max_baud(void) { return 1000000; }
static esp_err_t SERIAL_set_baud(int baud) { (void)baud; ++baud_changes; return fail(BAUD) ? ESP_FAIL : ESP_OK; }
static void stratum_minimal_pause_work(void) { ++pauses; work_paused = true; }
static void stratum_minimal_resume_work(void) { ++resumes; work_paused = false; }
static bool stratum_minimal_work_paused(void) { return work_paused; }
static bool stratum_minimal_wait_for_asic_rx_idle(uint32_t timeout_ms) {
    assert(work_paused && !g_regulator_enabled);
    if (idle_wait_ticks > timeout_ms) { tick += timeout_ms; return false; }
    tick += idle_wait_ticks; return true;
}
static void stratum_minimal_reset_hashrate(void) {}
static void reset_mining_telemetry_gate(void) {}
static void reset_domain_reboot_recovery(void) {}
static void remember_fault_operating_settings(GlobalState *s) { (void)s; }
static esp_err_t take_asic_transition_lock(TickType_t timeout) { (void)timeout; assert(locks == 0); ++locks; return ESP_OK; }
static void give_asic_transition_lock(void) { assert(locks == 1); --locks; }

/* HARDWARE_UNDER_TEST */

static GlobalState state;
static void reset(enum failure kind, int count) {
    state = (GlobalState){.ASIC_TASK_MODULE.active_jobs = &state, .valid_jobs = &state};
    tick = off_tick = 0; g_regulator_enabled = g_tps546_ready = off_recorded = false;
    work_paused = false;
    g_chip_count = g_commanded_voltage_mv = 0;
    records = shutdowns = attempts = serial_inits = detections = baud_changes = flushes = 0;
    pauses = resumes = locks = validation_calls = safety_validation = 0;
    idle_wait_ticks = 0; measured_temp = 55; tps_inits = reset_pulses = 0;
    failing = kind; failures_left = count; status[0] = last_record[0] = '\0';
}
int main(void) {
    reset(NONE, 0); state.ASIC_initalized = true; g_regulator_enabled = true;
    state.POWER_MANAGEMENT_MODULE.actual_frequency = 700;
    bitaxe_gamma602_log_read_retry(&state, "ASIC chip-ID read retrying");
    bitaxe_gamma602_log_read_retry(&state, "ASIC chip-ID read recovered");
    assert(records == 2 && strstr(first_record, "actual 700 MHz"));
    assert(strstr(last_record, "read recovered"));
    assert(state.ASIC_initalized && g_regulator_enabled && shutdowns == 0 && tps_inits == 0);
    assert(!state.SYSTEM_MODULE.hardware_fault && state.SYSTEM_MODULE.hardware_fault_msg[0] == '\0');

    const enum failure failures[] = {NO_CHIP, SERIAL, BAUD, TPS_INIT, RESET, FAN_INIT, TPS_SET, MONITOR};
    for (unsigned i = 0; i < sizeof(failures) / sizeof(failures[0]); ++i) {
        reset(failures[i], 1);
        assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_FAIL);
        assert(attempts == 1 && shutdowns == 1 && records == 1);
        assert(tps_inits <= 1 && detections <= 1 && reset_pulses <= 1);
        assert(!state.ASIC_initalized && !g_regulator_enabled && !state.SYSTEM_MODULE.hardware_fault);
        assert(state.SYSTEM_MODULE.hardware_fault_msg[0] != '\0' && !strcmp(status, "asic off"));
    }

    // Exhausted chip-ID reads cause one final shutdown. A subsequent manual On
    // may succeed; no second regulator initialization happens automatically.
    reset(NO_CHIP, 1);
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_FAIL);
    assert(tps_inits == 1 && detections == 1 && reset_pulses == 1 && shutdowns == 1);
    assert(tick >= 5000 && strstr(last_record, "no BM1370 detected"));
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_OK);
    assert(tps_inits == 2 && detections == 2 && reset_pulses == 2 && shutdowns == 1);
    assert(state.ASIC_initalized && g_regulator_enabled && state.SYSTEM_MODULE.hardware_fault_msg[0] == '\0');
    assert(records == 1); // Manual recovery does not erase the original event.

    reset(NONE, 0); idle_wait_ticks = ASIC_RX_DRAIN_TIMEOUT_MS + 1000;
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_ERR_TIMEOUT);
    assert(tick == ASIC_RX_DRAIN_TIMEOUT_MS && attempts == 1 && tps_inits == 0 && detections == 0);
    assert(!g_regulator_enabled && !state.ASIC_initalized && records == 1);
    assert(strstr(state.SYSTEM_MODULE.hardware_fault_msg, "UART receive did not become idle"));
    reset(NONE, 0); idle_wait_ticks = 10000;
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_OK);
    assert(tps_inits == 1 && detections == 1 && reset_pulses == 1 && shutdowns == 0);

    // A hard safety fault on either side of the handshake is never retried.
    for (unsigned where = 1; where <= 2; ++where) {
        reset(NONE, 0); safety_validation = where;
        assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_FAIL);
        assert(attempts == 1 && shutdowns == 1 && state.SYSTEM_MODULE.hardware_fault);
        assert(!state.ASIC_initalized && !g_regulator_enabled && !strcmp(status, "regulator shutdown"));
        assert(!strcmp(state.SYSTEM_MODULE.hardware_fault_msg, "TPS safety fault"));
        assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_ERR_INVALID_STATE);
        assert(attempts == 1 && tps_inits == 1);
    }
    reset(NONE, 0); measured_temp = 70;
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_FAIL);
    assert(attempts == 1 && state.SYSTEM_MODULE.hardware_fault && !g_regulator_enabled);
    assert(strstr(state.SYSTEM_MODULE.hardware_fault_msg, "70.0C"));
    reset(NONE, 0); measured_temp = 127;
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_OK);
    assert(!state.SYSTEM_MODULE.hardware_fault);

    reset(NONE, 0); state.POWER_MANAGEMENT_MODULE.actual_frequency = 700;
    assert(shutdown_and_return(&state, "hardware failed", ESP_FAIL) == ESP_FAIL);
    assert(strstr(first_record, "actual 700 MHz"));
    assert(state.POWER_MANAGEMENT_MODULE.actual_frequency == 0);
    reset(NONE, 0); work_paused = true;
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_OK);
    assert(pauses == 1 && resumes == 0 && work_paused);

    reset(NONE, 0); state.valid_jobs = NULL;
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_ERR_NO_MEM);
    assert(tps_inits == 0 && !g_regulator_enabled && strstr(state.SYSTEM_MODULE.hardware_fault_msg, "job tables missing"));
    reset(FAN_START, 1);
    assert(bitaxe_gamma602_set_asic_power(&state, true, true) == ESP_FAIL);
    assert(tps_inits == 0 && !g_regulator_enabled && !strcmp(status, "asic off"));
    assert(strstr(state.SYSTEM_MODULE.hardware_fault_msg, "fan start failed"));

    puts("Hardware startup/error lifecycle tests passed");
}
