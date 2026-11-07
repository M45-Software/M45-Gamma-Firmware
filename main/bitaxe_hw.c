#include "bitaxe_hw.h"

#include "bitaxe_fan.h"
#include <float.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "bm1370.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "asic_frequency_transition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bitaxe.h"
#include "m45_config.h"
#include "m45_auto_clock_voltage.h"
#include "m45_fault_log.h"
#include "m45_fault_context.h"
#include "m45_telemetry_extrema_store.h"
#include "mining.h"
#include "asic_serial.h"
#include "stratum_minimal.h"
#include "tps546.h"
#include "utils.h"

#define ASIC_RESET_GPIO CONFIG_GPIO_ASIC_RESET
#define ASIC_JOB_SLOTS 128
#define TPS546_STATUS_FAULT_MASK                                                                  \
    (TPS546_STATUS_OFF | TPS546_STATUS_VOUT_OV | TPS546_STATUS_IOUT_OC | TPS546_STATUS_VIN_UV |   \
     TPS546_STATUS_TEMP | TPS546_STATUS_PGOOD)
#define TPS546_VOUT_TOLERANCE_VOLTS 0.075f
#define TPS546_MONITOR_INTERVAL_MS 250
#define ASIC_FAN_MONITOR_INTERVAL_MS 250
// Auto-clock retains configuration frames while a TPS update retries reads and
// persists diagnostics. Leave headroom for the driver, logging, and NVS calls.
#define TPS546_POWER_MONITOR_STACK_BYTES 16384
#define TPS546_POWER_WINDOW_SAMPLES 16
#define TPS546_OUTPUT_SETTLE_INITIAL_MS 20
#define TPS546_OUTPUT_SETTLE_POLL_MS 10
#define TPS546_OUTPUT_SETTLE_TIMEOUT_MS 300
#define ASIC_FREQUENCY_SETTLE_MS 50
#define ASIC_POWER_OFF_MIN_MS 1000
#define ASIC_RX_DRAIN_TIMEOUT_MS 11000
#define ASIC_TEMP_STARTUP_GRACE_MS 3000
#define ASIC_TEMP_NO_READING_C 127.0f
#define ASIC_TEMP_NO_READING_LOG_MS 5000
#define ASIC_TEMP_UNAVAILABLE_TIMEOUT_US 5000000
#define ASIC_TEMP_VOLTAGE_UPDATE_DEADBAND_MV 5
#define AUTO_CLOCK_INTERVAL_MS 5000
#define AUTO_CLOCK_PRESET_MIN_MHZ 50
#define AUTO_CLOCK_PRESET_MAX_MHZ 1200
#define AUTO_CLOCK_PRESET_STEP_MHZ 25
#define AUTO_CLOCK_THERMAL_R_C_PER_W 1.4f
#define AUTO_CLOCK_TEMP_SAFETY_MARGIN_C 0.5f
#define AUTO_CLOCK_UP_TEMP_HYSTERESIS_C 2.0f
#define AUTO_CLOCK_DOWN_TEMP_HYSTERESIS_C 1.5f
#define AUTO_CLOCK_UP_POWER_HEADROOM_RATIO 0.97f
#define AUTO_CLOCK_UP_CURRENT_HEADROOM_RATIO 0.90f
#define AUTO_CLOCK_UP_VR_TEMP_HEADROOM_C 5.0f
#define AUTO_CLOCK_HOT_MARGIN_C 2.0f
#define AUTO_CLOCK_FAILSAFE_MARGIN_C 1.0f
#define AUTO_CLOCK_UP_DWELL_TICKS 3
#define AUTO_CLOCK_DOWN_DWELL_TICKS 2
#define AUTO_CLOCK_MAX_STEPS_UP 1
#define AUTO_CLOCK_MAX_STEPS_DOWN 3
#define AUTO_CLOCK_MAX_STEPS_DOWN_HOT 4
#define AUTO_CLOCK_STARTUP_MINING_STABLE_MS 30000
#define DOMAIN_REBOOT_CHECK_INTERVAL_MS 15000
#define DOMAIN_REBOOT_STARTUP_GRACE_MS 60000
#define DOMAIN_REBOOT_RECOVERY_MS 60000
#define DOMAIN_REBOOT_COOLDOWN_MS 300000
#define DOMAIN_REBOOT_POWER_CYCLE_OFF_MS 1000
#define DOMAIN_REBOOT_MIN_EXPECTED_RATIO 0.75
static const char *TAG = "bitaxe_hw";
static uint8_t g_chip_count = 0;
static char g_hw_status[64] = "boot";
static bool g_i2c_ready = false;
static bool g_power_monitor_started = false;
static bool g_regulator_enabled = false;
static bool g_tps546_ready = false;
static bool g_asic_power_off_recorded = false;
static TickType_t g_asic_power_off_tick;
static portMUX_TYPE g_asic_power_off_mux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t g_asic_transition_lock;
static portMUX_TYPE g_asic_transition_lock_init_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t g_commanded_voltage_mv = 0;
static TickType_t g_asic_temp_grace_until;
static int64_t g_asic_temp_unavailable_since_us = -1;
static TickType_t g_asic_temp_no_reading_log_next;
static TickType_t g_auto_clock_mining_ready_since;
static uint32_t g_mining_gate_job_sent_baseline;
static uint32_t g_mining_gate_valid_nonce_baseline;
static portMUX_TYPE g_power_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_auto_clock_status_lock = portMUX_INITIALIZER_UNLOCKED;
static bitaxe_gamma602_power_snapshot_t g_power_snapshot = {0};
// Diagnostic samples outlive a power transition; their ages distinguish old
// readings from a current measurement when a restart/read attempt fails.
static m45_fault_context_t g_fault_context;
static m45_telemetry_extrema_t g_telemetry_extrema;
static pthread_mutex_t g_extrema_persist_lock = PTHREAD_MUTEX_INITIALIZER;
static bool g_extrema_restore_pending;
_Static_assert(M45_FAULT_MESSAGE_MAX + 1 >=
                   M45_FAULT_CONTEXT_BYTES + M45_FAULT_REASON_MIN_BYTES,
               "fault record must fit the cause and operating context");
static bitaxe_gamma602_auto_clock_status_t g_auto_clock_status = {
    .thermal_resistance_c_per_w = AUTO_CLOCK_THERMAL_R_C_PER_W,
};
static TickType_t g_auto_clock_next_tick;
static m45_auto_clock_voltage_state_t g_auto_clock_voltage;
static uint8_t g_auto_clock_up_dwell_ticks;
static uint8_t g_auto_clock_down_dwell_ticks;
static TickType_t g_domain_reboot_next_tick;
static TickType_t g_domain_reboot_grace_until;
static TickType_t g_domain_reboot_cooldown_until;
static TickType_t g_domain_reboot_low_since[STRATUM_HASHRATE_MAX_ASICS]
                                            [STRATUM_HASH_DOMAIN_COUNT];
static float g_power_vin_window[TPS546_POWER_WINDOW_SAMPLES];
static float g_power_vout_window[TPS546_POWER_WINDOW_SAMPLES];
static float g_power_iout_window[TPS546_POWER_WINDOW_SAMPLES];
static float g_power_temp_window[TPS546_POWER_WINDOW_SAMPLES];
static float g_power_vin_sum;
static float g_power_vout_sum;
static float g_power_iout_sum;
static float g_power_temp_sum;
static uint8_t g_power_window_count;
static uint8_t g_power_window_next;

static esp_err_t bitaxe_gamma602_start_hardware_unlocked(GlobalState *state);
static esp_err_t bitaxe_gamma602_set_frequency_mhz_unlocked(GlobalState *state,
                                                            uint16_t frequency_mhz);
static esp_err_t bitaxe_gamma602_set_voltage_mv_for_config_unlocked(GlobalState *state,
                                                                    uint16_t voltage_mv,
                                                                    const m45_config_t *config);

static void set_hw_status(const char *status)
{
    strlcpy(g_hw_status, status, sizeof(g_hw_status));
}

static void record_asic_power_off(void)
{
    const TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL(&g_asic_power_off_mux);
    g_asic_power_off_tick = now;
    g_asic_power_off_recorded = true;
    taskEXIT_CRITICAL(&g_asic_power_off_mux);
}

static bool consume_asic_power_off_tick(TickType_t *off_tick)
{
    taskENTER_CRITICAL(&g_asic_power_off_mux);
    const bool recorded = g_asic_power_off_recorded;
    if (recorded && off_tick != NULL) {
        *off_tick = g_asic_power_off_tick;
    }
    g_asic_power_off_recorded = false;
    taskEXIT_CRITICAL(&g_asic_power_off_mux);
    return recorded;
}

static bool asic_temp_is_no_reading(float temp_c)
{
    return isfinite(temp_c) && temp_c > ASIC_TEMP_NO_READING_C;
}

static void log_asic_temp_no_reading(float temp_c)
{
    const TickType_t now = xTaskGetTickCount();
    if (g_asic_temp_no_reading_log_next != 0 &&
        (int32_t)(now - g_asic_temp_no_reading_log_next) < 0) {
        return;
    }

    ESP_LOGW(TAG, "ignoring ASIC temperature %.1f C as no reading", temp_c);
    g_asic_temp_no_reading_log_next =
        now + pdMS_TO_TICKS(ASIC_TEMP_NO_READING_LOG_MS);
}

static TPS546_CONFIG gamma_tps546_config_from(const m45_config_t *active)
{
    TPS546_CONFIG config = {0};
    config.TPS546_INIT_PHASE = TPS546_INIT_PHASE_SINGLE;
    config.TPS546_INIT_VIN_ON =
        active->safety_input_voltage_expected_min_mv / 1000.0f;
    config.TPS546_INIT_VIN_OFF = active->safety_input_voltage_min_mv / 1000.0f;
    config.TPS546_INIT_VIN_UV_WARN_LIMIT = 0.0f;
    config.TPS546_INIT_VIN_OV_FAULT_LIMIT =
        active->safety_input_voltage_max_mv / 1000.0f;
    config.TPS546_INIT_SCALE_LOOP = 0.25f;
    config.TPS546_INIT_VOUT_MIN = active->safety_asic_voltage_min_mv / 1000.0f;
    config.TPS546_INIT_VOUT_MAX = active->safety_asic_voltage_max_mv / 1000.0f;
    config.TPS546_INIT_VOUT_COMMAND =
        m45_config_effective_asic_voltage_mv(active) / 1000.0f;
    config.TPS546_INIT_IOUT_OC_WARN_LIMIT =
        active->safety_iout_warn_deciamps / 10.0f;
    config.TPS546_INIT_IOUT_OC_FAULT_LIMIT =
        active->safety_iout_fault_deciamps / 10.0f;
    config.TPS546_INIT_STACK_CONFIG = 0x0000;
    config.TPS546_INIT_SYNC_CONFIG = 0x10;
    return config;
}

static TPS546_CONFIG gamma_tps546_config(void)
{
    return gamma_tps546_config_from(m45_config_get());
}

static esp_err_t asic_reset_level(int level)
{
    ESP_RETURN_ON_ERROR(gpio_reset_pin(ASIC_RESET_GPIO), TAG, "reset pin reset failed");
    ESP_RETURN_ON_ERROR(gpio_set_direction(ASIC_RESET_GPIO, GPIO_MODE_OUTPUT), TAG,
                        "reset pin direction failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(ASIC_RESET_GPIO, level), TAG, "reset pin set failed");
    return ESP_OK;
}

static esp_err_t asic_reset_pulse(void)
{
    ESP_RETURN_ON_ERROR(asic_reset_level(0), TAG, "reset low failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(gpio_set_level(ASIC_RESET_GPIO, 1), TAG, "reset high failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static void persist_telemetry_extrema(void)
{
    pthread_mutex_lock(&g_extrema_persist_lock);
    if (g_extrema_restore_pending) {
        m45_telemetry_extrema_t saved;
        const esp_err_t restore_err = m45_telemetry_extrema_load(&saved);
        if (restore_err != ESP_OK) {
            pthread_mutex_unlock(&g_extrema_persist_lock);
            ESP_LOGW(TAG, "telemetry extremes restore pending: %s", esp_err_to_name(restore_err));
            return;
        }
        // Observations collected while storage was unavailable augment the
        // restored record; they must never replace older, more extreme values.
        portENTER_CRITICAL(&g_power_snapshot_lock);
        if (saved.vin_valid) {
            m45_telemetry_extrema_vin(&g_telemetry_extrema, saved.vin_min_v);
            m45_telemetry_extrema_vin(&g_telemetry_extrema, saved.vin_max_v);
        }
        if (saved.tps_temp_valid) {
            m45_telemetry_extrema_tps_temp(&g_telemetry_extrema, saved.tps_temp_max_c);
        }
        if (saved.asic_temp_valid) {
            m45_telemetry_extrema_asic_temp(&g_telemetry_extrema, saved.asic_temp_max_c);
        }
        portEXIT_CRITICAL(&g_power_snapshot_lock);
        g_extrema_restore_pending = false;
    }
    m45_telemetry_extrema_t extrema;
    bitaxe_gamma602_telemetry_extrema(&extrema);
    const esp_err_t err = m45_telemetry_extrema_save(&extrema);
    pthread_mutex_unlock(&g_extrema_persist_lock);
    if (err != ESP_OK) ESP_LOGW(TAG, "failed to save telemetry extremes: %s", esp_err_to_name(err));
}

static void extrema_persist_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        persist_telemetry_extrema();
    }
}

esp_err_t bitaxe_gamma602_telemetry_extrema_init(void)
{
    m45_telemetry_extrema_t extrema;
    esp_err_t err = m45_telemetry_extrema_load(&extrema);
    g_extrema_restore_pending = err != ESP_OK;
    if (err != ESP_OK) {
        // Bad/missing storage must not block hardware protection or startup.
        ESP_LOGW(TAG, "telemetry extremes unavailable: %s", esp_err_to_name(err));
    }
    portENTER_CRITICAL(&g_power_snapshot_lock);
    g_telemetry_extrema = extrema;
    portEXIT_CRITICAL(&g_power_snapshot_lock);
    ESP_RETURN_ON_ERROR(esp_register_shutdown_handler(persist_telemetry_extrema), TAG,
                        "register telemetry save on reboot failed");
    if (xTaskCreate(extrema_persist_task, "extrema_save", 4096, NULL,
                    tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        (void)esp_unregister_shutdown_handler(persist_telemetry_extrema);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t bitaxe_gamma602_clear_telemetry_extrema(void)
{
    // Serialize against background/reboot saves. Publish the cleared state only
    // after NVS commits, so failed clears preserve both RAM and saved records.
    pthread_mutex_lock(&g_extrema_persist_lock);
    const m45_telemetry_extrema_t empty = {0};
    const esp_err_t err = m45_telemetry_extrema_save(&empty);
    if (err == ESP_OK) {
        g_extrema_restore_pending = false;
        portENTER_CRITICAL(&g_power_snapshot_lock);
        g_telemetry_extrema = empty;
        portEXIT_CRITICAL(&g_power_snapshot_lock);
    }
    pthread_mutex_unlock(&g_extrema_persist_lock);
    return err;
}

static void remember_power_extrema(const TPS546_StatusSnapshot *snapshot)
{
    // Keep valid raw fields even when a later register read fails or a safety
    // limit is exceeded. Never record zero-filled unread fields as measurements.
    portENTER_CRITICAL(&g_power_snapshot_lock);
    if (snapshot->valid & (1U << TPS546_FIELD_VIN)) {
        m45_telemetry_extrema_vin(&g_telemetry_extrema, snapshot->read_vin);
    }
    if (snapshot->valid & (1U << TPS546_FIELD_TEMP)) {
        m45_telemetry_extrema_tps_temp(&g_telemetry_extrema, (float)snapshot->read_temp1);
    }
    portEXIT_CRITICAL(&g_power_snapshot_lock);
}

static void remember_fault_power_sample(const TPS546_StatusSnapshot *snapshot)
{
    if (snapshot->read_error != ESP_OK ||
        snapshot->valid != TPS546_SNAPSHOT_ALL_VALID) {
        return;
    }
    const uint64_t now_ms = (uint64_t)esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&g_power_snapshot_lock);
    m45_fault_context_remember_power(&g_fault_context, now_ms, snapshot->read_vin,
                                     snapshot->read_vout, snapshot->read_iout,
                                     (float)snapshot->read_temp1);
    portEXIT_CRITICAL(&g_power_snapshot_lock);
}

static void remember_fault_asic_temperature(float temp_c)
{
    const uint64_t now_ms = (uint64_t)esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&g_power_snapshot_lock);
    m45_fault_context_remember_asic_temp(&g_fault_context, now_ms, temp_c);
    m45_telemetry_extrema_asic_temp(&g_telemetry_extrema, temp_c);
    portEXIT_CRITICAL(&g_power_snapshot_lock);
}

static void store_power_snapshot(const TPS546_StatusSnapshot *snapshot)
{
    const bitaxe_gamma602_power_snapshot_t compact = {
        .valid = true,
        .status_word = snapshot->status_word,
        .operation = snapshot->operation,
        .vout_command = snapshot->vout_command,
        .read_vout = snapshot->read_vout,
        .read_vin = snapshot->read_vin,
        .read_iout = snapshot->read_iout,
        .read_temp_c = snapshot->read_temp1,
    };

    portENTER_CRITICAL(&g_power_snapshot_lock);
    g_power_snapshot = compact;
    portEXIT_CRITICAL(&g_power_snapshot_lock);
}

static void clear_power_snapshot(void)
{
    portENTER_CRITICAL(&g_power_snapshot_lock);
    memset(&g_power_snapshot, 0, sizeof(g_power_snapshot));
    portEXIT_CRITICAL(&g_power_snapshot_lock);
}

static void reset_power_average_window(void)
{
    memset(g_power_vin_window, 0, sizeof(g_power_vin_window));
    memset(g_power_vout_window, 0, sizeof(g_power_vout_window));
    memset(g_power_iout_window, 0, sizeof(g_power_iout_window));
    memset(g_power_temp_window, 0, sizeof(g_power_temp_window));
    g_power_vin_sum = 0.0f;
    g_power_vout_sum = 0.0f;
    g_power_iout_sum = 0.0f;
    g_power_temp_sum = 0.0f;
    g_power_window_count = 0;
    g_power_window_next = 0;
}

static bool tick_before(TickType_t now, TickType_t target)
{
    return target != 0 && (int32_t)(now - target) < 0;
}

static esp_err_t ensure_asic_transition_lock(void)
{
    if (g_asic_transition_lock != NULL) {
        return ESP_OK;
    }

    SemaphoreHandle_t lock = xSemaphoreCreateMutex();
    if (lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    taskENTER_CRITICAL(&g_asic_transition_lock_init_mux);
    if (g_asic_transition_lock == NULL) {
        g_asic_transition_lock = lock;
        lock = NULL;
    }
    taskEXIT_CRITICAL(&g_asic_transition_lock_init_mux);

    if (lock != NULL) {
        vSemaphoreDelete(lock);
    }
    return ESP_OK;
}

static esp_err_t take_asic_transition_lock(TickType_t timeout_ticks)
{
    ESP_RETURN_ON_ERROR(ensure_asic_transition_lock(), TAG,
                        "ASIC transition lock init failed");
    return xSemaphoreTake(g_asic_transition_lock, timeout_ticks) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

static void give_asic_transition_lock(void)
{
    if (g_asic_transition_lock != NULL) {
        xSemaphoreGive(g_asic_transition_lock);
    }
}

static void reset_auto_clock_control_state(void)
{
    memset(&g_auto_clock_voltage, 0, sizeof(g_auto_clock_voltage));
    g_auto_clock_mining_ready_since = 0;
    g_auto_clock_next_tick = 0;
    g_auto_clock_up_dwell_ticks = 0;
    g_auto_clock_down_dwell_ticks = 0;
}

static void reset_mining_telemetry_gate(void)
{
    reset_auto_clock_control_state();
    g_mining_gate_job_sent_baseline = stratum_minimal_job_sent_count();
    g_mining_gate_valid_nonce_baseline = stratum_minimal_valid_nonce_count();
}

void bitaxe_gamma602_reset_auto_clock_control(void)
{
    reset_auto_clock_control_state();
}

static bool mining_startup_gate_ready(const char **reason_out)
{
    const bool started = stratum_minimal_started();
    const bool connected = stratum_minimal_connected();
    const uint32_t jobs_sent = stratum_minimal_job_sent_count();
    const bool has_post_boot_work =
        (uint32_t)(jobs_sent - g_mining_gate_job_sent_baseline) > 0;
    const uint32_t valid_nonces = stratum_minimal_valid_nonce_count();
    const bool has_post_boot_nonce =
        (uint32_t)(valid_nonces - g_mining_gate_valid_nonce_baseline) > 0;
    const char *reason = "";

    if (!started) {
        reason = "waiting for stratum start";
    } else if (!connected) {
        reason = "waiting for stratum";
    } else if (!has_post_boot_work) {
        reason = "waiting for ASIC work";
    } else if (!has_post_boot_nonce) {
        reason = "waiting for ASIC results";
    } else {
        if (reason_out != NULL) {
            *reason_out = "";
        }
        return true;
    }

    if (reason_out != NULL) {
        *reason_out = reason;
    }
    return false;
}

static bool auto_clock_startup_gate_ready(const char **reason_out)
{
    const char *reason = NULL;
    if (!mining_startup_gate_ready(&reason)) {
        g_auto_clock_mining_ready_since = 0;
        if (reason_out != NULL) {
            *reason_out = reason;
        }
        return false;
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t stable_ticks =
        pdMS_TO_TICKS(AUTO_CLOCK_STARTUP_MINING_STABLE_MS);
    if (g_auto_clock_mining_ready_since == 0) {
        g_auto_clock_mining_ready_since = now;
    }
    if ((TickType_t)(now - g_auto_clock_mining_ready_since) < stable_ticks) {
        static char stable_reason[64];
        const uint32_t waited_ms =
            pdTICKS_TO_MS(now - g_auto_clock_mining_ready_since);
        snprintf(stable_reason, sizeof(stable_reason),
                 "waiting for stable mining %lu/%u sec",
                 (unsigned long)(waited_ms / 1000),
                 (unsigned)(AUTO_CLOCK_STARTUP_MINING_STABLE_MS / 1000));
        if (reason_out != NULL) {
            *reason_out = stable_reason;
        }
        return false;
    }

    if (reason_out != NULL) {
        *reason_out = "";
    }
    return true;
}

static bool mining_telemetry_ready(stratum_minimal_stats_t *stats_out,
                                   const char **reason_out)
{
    if (!mining_startup_gate_ready(reason_out)) {
        return false;
    }

    stratum_minimal_stats_t stats;
    stratum_minimal_get_stats(&stats);
    if (stats_out != NULL) {
        *stats_out = stats;
    }

    if (reason_out != NULL) {
        *reason_out = "";
    }
    return true;
}

static void clear_domain_reboot_low_since(void)
{
    memset(g_domain_reboot_low_since, 0, sizeof(g_domain_reboot_low_since));
}

static void reset_domain_reboot_recovery(void)
{
    g_domain_reboot_next_tick = 0;
    clear_domain_reboot_low_since();
}

static void reset_domain_reboot_watchdog(void)
{
    reset_domain_reboot_recovery();
    g_domain_reboot_cooldown_until = 0;
}

static bool domain_reboot_recovery_pending(void)
{
    const m45_config_t *config = m45_config_get();
    if (config == NULL || !config->auto_domain_reboot_enabled) {
        return false;
    }

    for (uint8_t asic = 0; asic < STRATUM_HASHRATE_MAX_ASICS; ++asic) {
        for (uint8_t domain = 0; domain < STRATUM_HASH_DOMAIN_COUNT; ++domain) {
            if (g_domain_reboot_low_since[asic][domain] != 0) {
                return true;
            }
        }
    }
    return false;
}

static void update_auto_clock_status(bool enabled, bool active, uint16_t target_frequency_mhz,
                                     uint16_t target_voltage_mv, uint16_t next_up_frequency_mhz,
                                     float power_now_w, float power_target_w,
                                     float next_up_power_w, float output_current_ceiling_a,
                                     float next_up_output_current_a, bool input_voltage_limited,
                                     bool output_current_limited, bool vr_temp_limited,
                                     bool power_limited, bool temperature_limited,
                                     const char *hold_reason)
{
    bitaxe_gamma602_auto_clock_status_t status = {
        .enabled = enabled,
        .active = active,
        .target_frequency_mhz = target_frequency_mhz,
        .target_voltage_mv = target_voltage_mv,
        .next_up_frequency_mhz = next_up_frequency_mhz,
        .power_now_w = power_now_w,
        .power_target_w = power_target_w,
        .next_up_power_w = next_up_power_w,
        .thermal_resistance_c_per_w = AUTO_CLOCK_THERMAL_R_C_PER_W,
        .output_current_ceiling_a = output_current_ceiling_a,
        .next_up_output_current_a = next_up_output_current_a,
        .input_voltage_limited = input_voltage_limited,
        .output_current_limited = output_current_limited,
        .vr_temp_limited = vr_temp_limited,
        .power_limited = power_limited,
        .temperature_limited = temperature_limited,
    };
    if (hold_reason != NULL) {
        strlcpy(status.hold_reason, hold_reason, sizeof(status.hold_reason));
    }

    portENTER_CRITICAL(&g_auto_clock_status_lock);
    g_auto_clock_status = status;
    portEXIT_CRITICAL(&g_auto_clock_status_lock);
}

static uint16_t auto_clock_preset_frequency(uint8_t index)
{
    return AUTO_CLOCK_PRESET_MIN_MHZ +
           (uint16_t)index * AUTO_CLOCK_PRESET_STEP_MHZ;
}

static uint8_t auto_clock_preset_count(void)
{
    return (uint8_t)(((AUTO_CLOCK_PRESET_MAX_MHZ - AUTO_CLOCK_PRESET_MIN_MHZ) /
                      AUTO_CLOCK_PRESET_STEP_MHZ) +
                     1);
}

static uint8_t auto_clock_index_for_frequency(uint16_t frequency_mhz)
{
    if (frequency_mhz <= AUTO_CLOCK_PRESET_MIN_MHZ) {
        return 0;
    }
    if (frequency_mhz >= AUTO_CLOCK_PRESET_MAX_MHZ) {
        return (uint8_t)(auto_clock_preset_count() - 1);
    }

    const uint16_t offset_mhz = frequency_mhz - AUTO_CLOCK_PRESET_MIN_MHZ;
    const uint16_t rounded =
        (offset_mhz + (AUTO_CLOCK_PRESET_STEP_MHZ / 2)) / AUTO_CLOCK_PRESET_STEP_MHZ;
    return (uint8_t)rounded;
}

static uint16_t auto_clock_base_voltage_mv(uint16_t frequency_mhz,
                                           int16_t voltage_offset_mv)
{
    int32_t base_mv = 0;
    if (frequency_mhz == CONFIG_M45_BITAXE_ASIC_FREQUENCY_MHZ) {
        base_mv = CONFIG_M45_BITAXE_ASIC_VOLTAGE_MV;
    } else {
        base_mv = (int32_t)lroundf((1150.0f + (0.35f * ((float)frequency_mhz - 850.0f))) /
                                   5.0f) *
                  5;
    }

    base_mv += voltage_offset_mv;
    if (base_mv < 500) {
        return 500;
    }
    if (base_mv > 1370) {
        return 1370;
    }
    return (uint16_t)base_mv;
}

static bool auto_clock_candidate_voltage(const m45_config_t *config,
                                         uint16_t frequency_mhz,
                                         float asic_temp_c,
                                         uint16_t *base_mv,
                                         uint16_t *target_mv)
{
    m45_config_t candidate = *config;
    candidate.overclock_enabled = true;
    candidate.asic_frequency_mhz = frequency_mhz;
    candidate.asic_voltage_mv =
        auto_clock_base_voltage_mv(frequency_mhz, config->overclock_voltage_offset_mv);
    if (candidate.asic_voltage_mv < candidate.safety_asic_voltage_min_mv ||
        candidate.asic_voltage_mv >= candidate.safety_asic_voltage_max_mv) {
        return false;
    }

    const uint16_t compensated_mv =
        m45_config_effective_asic_voltage_mv_for_temp(&candidate, asic_temp_c);
    if (compensated_mv < candidate.safety_asic_voltage_min_mv ||
        compensated_mv >= candidate.safety_asic_voltage_max_mv) {
        return false;
    }

    if (base_mv != NULL) {
        *base_mv = candidate.asic_voltage_mv;
    }
    if (target_mv != NULL) {
        *target_mv = compensated_mv;
    }
    return true;
}

static uint16_t current_auto_clock_frequency_mhz(GlobalState *state,
                                                const m45_config_t *config)
{
    const float actual_mhz =
        state != NULL ? state->POWER_MANAGEMENT_MODULE.actual_frequency : 0.0f;
    if (isfinite(actual_mhz) && actual_mhz > 0.0f && actual_mhz <= 65535.0f) {
        return (uint16_t)lroundf(actual_mhz);
    }
    return m45_config_effective_asic_frequency_mhz(config);
}

static uint16_t current_auto_clock_voltage_mv(const m45_config_t *config,
                                             float asic_temp_c)
{
    if (g_commanded_voltage_mv > 0) {
        return g_commanded_voltage_mv;
    }
    if (isfinite(asic_temp_c) && asic_temp_c > 0.0f) {
        return m45_config_effective_asic_voltage_mv_for_temp(config, asic_temp_c);
    }
    return m45_config_effective_asic_voltage_mv(config);
}

static float auto_clock_upshift_current_ceiling_a(const m45_config_t *config)
{
    const float warn_a = config->safety_iout_warn_deciamps / 10.0f;
    const float fault_headroom_a =
        (config->safety_iout_fault_deciamps / 10.0f) *
        AUTO_CLOCK_UP_CURRENT_HEADROOM_RATIO;
    if (warn_a > 0.0f && fault_headroom_a > 0.0f) {
        return fminf(warn_a, fault_headroom_a);
    }
    return fmaxf(warn_a, fault_headroom_a);
}

static float auto_clock_upshift_vr_temp_ceiling_c(const m45_config_t *config)
{
    const float expected_c = (float)config->safety_tps546_temp_expected_max_c;
    const float max_headroom_c =
        fmaxf(0.0f, (float)config->safety_tps546_temp_max_c -
                        AUTO_CLOCK_UP_VR_TEMP_HEADROOM_C);
    if (expected_c > 0.0f && max_headroom_c > 0.0f) {
        return fminf(expected_c, max_headroom_c);
    }
    return fmaxf(expected_c, max_headroom_c);
}

static uint8_t auto_clock_clamp_toward(uint8_t current_index, uint8_t target_index,
                                       uint8_t max_steps_up, uint8_t max_steps_down)
{
    if (target_index > current_index) {
        const uint8_t delta = target_index - current_index;
        return current_index + (delta > max_steps_up ? max_steps_up : delta);
    }
    if (target_index < current_index) {
        const uint8_t delta = current_index - target_index;
        return current_index - (delta > max_steps_down ? max_steps_down : delta);
    }
    return current_index;
}

static esp_err_t auto_clock_apply_preset(GlobalState *state, const m45_config_t *current_config,
                                         uint16_t next_frequency_mhz,
                                         uint16_t next_base_voltage_mv,
                                         uint16_t next_target_voltage_mv)
{
    esp_err_t lock_err = take_asic_transition_lock(0);
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    m45_config_t next_config = *current_config;
    next_config.auto_clock_enabled = true;
    next_config.overclock_enabled = true;
    next_config.asic_frequency_mhz = next_frequency_mhz;
    next_config.asic_voltage_mv = next_base_voltage_mv;

    const uint16_t current_frequency_mhz =
        current_auto_clock_frequency_mhz(state, current_config);
    const uint16_t current_voltage_mv =
        current_auto_clock_voltage_mv(current_config, 0.0f);
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, "auto clock preset: %u MHz / %u mV -> %u MHz / %u mV",
             current_frequency_mhz, current_voltage_mv,
             next_frequency_mhz, next_base_voltage_mv);

    const bool frequency_changing = next_frequency_mhz != current_frequency_mhz;
    const bool voltage_changing = next_target_voltage_mv != current_voltage_mv;
    const bool raise_voltage_before_frequency =
        frequency_changing && voltage_changing &&
        next_frequency_mhz > current_frequency_mhz &&
        next_target_voltage_mv > current_voltage_mv;

    if (raise_voltage_before_frequency) {
        err = bitaxe_gamma602_set_voltage_mv_for_config_unlocked(
            state, next_target_voltage_mv, &next_config);
    }
    if (err == ESP_OK && frequency_changing) {
        stratum_minimal_pause_work();
        err = bitaxe_gamma602_set_frequency_mhz_unlocked(state, next_frequency_mhz);
        stratum_minimal_resume_work();
    }
    if (err == ESP_OK && voltage_changing && !raise_voltage_before_frequency) {
        err = bitaxe_gamma602_set_voltage_mv_for_config_unlocked(
            state, next_target_voltage_mv, &next_config);
    }

    if (err != ESP_OK) {
        give_asic_transition_lock();
        return err;
    }

    give_asic_transition_lock();
    return ESP_OK;
}

static void remember_fault_operating_settings(GlobalState *state)
{
    const uint64_t now_ms = (uint64_t)esp_timer_get_time() / 1000;
    const uint16_t requested_mv = m45_config_effective_asic_voltage_mv(m45_config_get());
    portENTER_CRITICAL(&g_power_snapshot_lock);
    m45_fault_context_remember_settings(&g_fault_context, now_ms,
                                        state->POWER_MANAGEMENT_MODULE.actual_frequency,
                                        state->POWER_MANAGEMENT_MODULE.frequency_value,
                                        requested_mv, g_commanded_voltage_mv);
    portEXIT_CRITICAL(&g_power_snapshot_lock);
}

static void update_power_state(GlobalState *state, const TPS546_StatusSnapshot *snapshot)
{
    remember_fault_operating_settings(state);
    state->POWER_MANAGEMENT_MODULE.voltage = snapshot->read_vin;
    state->POWER_MANAGEMENT_MODULE.current = snapshot->read_iout;
    state->POWER_MANAGEMENT_MODULE.core_voltage = snapshot->read_vout;
    state->POWER_MANAGEMENT_MODULE.power = snapshot->read_vout * snapshot->read_iout;
    state->POWER_MANAGEMENT_MODULE.vr_temp = (float)snapshot->read_temp1;
}

static void update_power_average(const TPS546_StatusSnapshot *raw, TPS546_StatusSnapshot *averaged)
{
    const uint8_t index = g_power_window_next;
    if (g_power_window_count == TPS546_POWER_WINDOW_SAMPLES) {
        g_power_vin_sum -= g_power_vin_window[index];
        g_power_vout_sum -= g_power_vout_window[index];
        g_power_iout_sum -= g_power_iout_window[index];
        g_power_temp_sum -= g_power_temp_window[index];
    } else {
        ++g_power_window_count;
    }

    g_power_vin_window[index] = raw->read_vin;
    g_power_vout_window[index] = raw->read_vout;
    g_power_iout_window[index] = raw->read_iout;
    g_power_temp_window[index] = (float)raw->read_temp1;
    g_power_vin_sum += g_power_vin_window[index];
    g_power_vout_sum += g_power_vout_window[index];
    g_power_iout_sum += g_power_iout_window[index];
    g_power_temp_sum += g_power_temp_window[index];
    g_power_window_next = (uint8_t)((index + 1) % TPS546_POWER_WINDOW_SAMPLES);

    const float samples = (float)g_power_window_count;
    *averaged = *raw;
    averaged->read_vin = g_power_vin_sum / samples;
    averaged->read_vout = g_power_vout_sum / samples;
    averaged->read_iout = g_power_iout_sum / samples;
    averaged->read_temp1 = (int)lroundf(g_power_temp_sum / samples);
}


static esp_err_t shutdown_regulator(GlobalState *state, const char *reason)
{
    ESP_LOGE(TAG, "%s; turning TPS546 output off", reason);
    set_hw_status("regulator shutdown");
    g_regulator_enabled = false;
    g_commanded_voltage_mv = 0;
    g_chip_count = 0;
    state->POWER_MANAGEMENT_MODULE.actual_frequency = 0.0f;
    clear_power_snapshot();

    esp_err_t reset_err = asic_reset_level(0);
    if (reset_err != ESP_OK) {
        ESP_LOGW(TAG, "failed to hold ASIC reset low during regulator shutdown: %s",
                 esp_err_to_name(reset_err));
    }

    // Stop the ASIC before any I2C access, which can wait for bus retries.
    bitaxe_fan_force_max_if_allowed(state, reason);

    if (!g_tps546_ready) {
        record_asic_power_off();
        return ESP_OK;
    }
    const esp_err_t err = TPS546_set_vout(0.0f);
    record_asic_power_off();
    return err;
}

static void flag_safety_fault(GlobalState *state, const char *reason)
{
    state->SYSTEM_MODULE.power_fault = 1;
    state->SYSTEM_MODULE.hardware_fault = true;
    strlcpy(state->SYSTEM_MODULE.hardware_fault_msg, reason,
            sizeof(state->SYSTEM_MODULE.hardware_fault_msg));
    state->ASIC_initalized = false;
    bitaxe_gamma602_clear_jobs(state);
}

static void format_fault_context(GlobalState *state, const char *reason,
                                  char *dst, size_t size)
{
    remember_fault_operating_settings(state);
    m45_fault_context_t context;
    portENTER_CRITICAL(&g_power_snapshot_lock);
    context = g_fault_context;
    portEXIT_CRITICAL(&g_power_snapshot_lock);
    if (!context.operating_settings_valid) {
        context.requested_frequency_mhz = state->POWER_MANAGEMENT_MODULE.frequency_value;
        context.requested_voltage_mv =
            m45_config_effective_asic_voltage_mv(m45_config_get());
    }
    m45_fault_context_format(&context, (uint64_t)esp_timer_get_time() / 1000,
                             reason, dst, size);
}

void bitaxe_gamma602_log_read_retry(GlobalState *state, const char *reason)
{
    if (state == NULL || reason == NULL) {
        return;
    }
    char message[M45_FAULT_MESSAGE_MAX + 1];
    format_fault_context(state, reason, message, sizeof(message));
    ESP_LOGW(TAG, "%s", message);
    (void)m45_fault_log_record(message);
}

static void safety_shutdown(GlobalState *state, char *reason, size_t reason_size)
{
    // Both callers own a full-size mutable fault buffer, so enrich in place
    // before shutdown clears telemetry without another 1 KiB stack allocation.
    format_fault_context(state, reason, reason, reason_size);
    flag_safety_fault(state, reason);
    esp_err_t shutdown_err = shutdown_regulator(state, reason);
    if (shutdown_err != ESP_OK) {
        ESP_LOGE(TAG, "regulator shutdown failed after %s: %s", reason,
                 esp_err_to_name(shutdown_err));
    }
    set_hw_status("safety shutdown");
    // Flash writes follow reset/output-off, so saving cannot delay protection.
    persist_telemetry_extrema();
    if (m45_fault_log_record(reason) != ESP_OK) {
        ESP_LOGW(TAG, "failed to persist safety fault");
    }
}

static void safety_shutdownf(GlobalState *state, const char *fmt, ...)
{
    char message[M45_FAULT_MESSAGE_MAX + 1];
    va_list args;
    va_start(args, fmt);
    vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);
    safety_shutdown(state, message, sizeof(message));
}

static esp_err_t shutdown_and_return(GlobalState *state, const char *reason, esp_err_t result)
{
    char message[M45_FAULT_MESSAGE_MAX + 1];
    snprintf(message, sizeof(message), "%s: %s (0x%x)", reason,
             esp_err_to_name(result), (unsigned)result);
    format_fault_context(state, message, message, sizeof(message));
    // Keep the failing stage instead of replacing it with "hardware init failed".
    if (!state->SYSTEM_MODULE.hardware_fault) {
        strlcpy(state->SYSTEM_MODULE.hardware_fault_msg, message,
                sizeof(state->SYSTEM_MODULE.hardware_fault_msg));
    }
    state->ASIC_initalized = false;
    bitaxe_gamma602_clear_jobs(state);
    esp_err_t shutdown_err = shutdown_regulator(state, message);
    if (shutdown_err != ESP_OK) {
        ESP_LOGE(TAG, "regulator shutdown failed after %s: %s (0x%x)", message,
                 esp_err_to_name(shutdown_err), (unsigned)shutdown_err);
    }
    (void)m45_fault_log_record(message);
    return result;
}

static esp_err_t fail_regulator_safety(GlobalState *state,
                                       const TPS546_StatusSnapshot *snapshot,
                                       const char *fmt, ...)
{
    if (snapshot != NULL) {
        remember_power_extrema(snapshot);
    }
    char message[M45_FAULT_MESSAGE_MAX + 1];
    va_list args;
    va_start(args, fmt);
    vsnprintf(message, 256, fmt, args);
    va_end(args);
    size_t used = strlen(message);
    snprintf(message + used, sizeof(message) - used, "; ");
    used = strlen(message);
    TPS546_format_snapshot(snapshot, message + used, sizeof(message) - used);
    safety_shutdown(state, message, sizeof(message));
    return ESP_FAIL;
}

static esp_err_t validate_regulator_safety(GlobalState *state,
                                           const TPS546_StatusSnapshot *snapshot,
                                           float target_vout,
                                           bool require_target_vout)
{
    bitaxe_gamma602_safety_limits_t limits;
    bitaxe_gamma602_safety_limits(&limits);

    if ((snapshot->valid & (1U << TPS546_FIELD_STATUS)) &&
        (snapshot->status_word & TPS546_STATUS_FAULT_MASK)) {
        return fail_regulator_safety(
            state, snapshot,
            "TPS546 status fault (limits: VIN %.3f-%.3fV, IOUT warn/fault %.1f/%.1fA, VR %.0fC)",
            limits.input_voltage_min_v, limits.input_voltage_max_v, limits.iout_warn_a, limits.iout_fault_a,
            limits.tps546_temp_max_c);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_STATUS)) &&
        (snapshot->valid & (1U << TPS546_FIELD_INPUT_STATUS)) &&
        (snapshot->status_word & TPS546_STATUS_INPUT) &&
        (snapshot->st_input & TPS546_STATUS_VIN_OVF)) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VIN overvoltage flag; configured limit %.3fV",
            limits.input_voltage_max_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_OPERATION)) &&
        (snapshot->operation & OPERATION_ON) == 0) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 operation is not ON: OP=0x%02x, expected ON bit=0x%02x",
            snapshot->operation, OPERATION_ON);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VIN)) &&
        snapshot->read_vin < limits.input_voltage_min_v) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VIN %.3fV < minimum %.3fV", snapshot->read_vin,
            limits.input_voltage_min_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VIN)) &&
        snapshot->read_vin >= limits.input_voltage_max_v) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VIN %.3fV >= maximum %.3fV", snapshot->read_vin,
            limits.input_voltage_max_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VOUT)) &&
        snapshot->read_vout < limits.asic_voltage_min_v) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VOUT %.3fV < minimum %.3fV while enabled", snapshot->read_vout,
            limits.asic_voltage_min_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VOUT)) &&
        snapshot->read_vout >= limits.asic_voltage_max_v) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VOUT %.3fV >= maximum %.3fV", snapshot->read_vout,
            limits.asic_voltage_max_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VOUT)) && require_target_vout &&
        fabsf(snapshot->read_vout - target_vout) > TPS546_VOUT_TOLERANCE_VOLTS) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VOUT %.3fV; target %.3fV +/- %.3fV", snapshot->read_vout, target_vout,
            TPS546_VOUT_TOLERANCE_VOLTS);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_TEMP)) &&
        (float)snapshot->read_temp1 >= limits.tps546_temp_max_c) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VR temperature %dC >= maximum %.0fC", snapshot->read_temp1,
            limits.tps546_temp_max_c);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_IOUT)) &&
        snapshot->read_iout >= limits.iout_fault_a) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 IOUT %.3fA >= maximum %.3fA", snapshot->read_iout,
            limits.iout_fault_a);
    }

    return ESP_OK;
}

static esp_err_t unavailable_asic_temperature(GlobalState *state, float temp_c)
{
    log_asic_temp_no_reading(temp_c);
    if (domain_reboot_recovery_pending() ||
        xTaskGetTickCount() < g_asic_temp_grace_until) {
        g_asic_temp_unavailable_since_us = -1;
        return ESP_ERR_INVALID_STATE;
    }
    const int64_t now_us = esp_timer_get_time();
    if (g_asic_temp_unavailable_since_us < 0) {
        g_asic_temp_unavailable_since_us = now_us;
    }
    if (now_us - g_asic_temp_unavailable_since_us >= ASIC_TEMP_UNAVAILABLE_TIMEOUT_US) {
        safety_shutdownf(state, "ASIC temperature unavailable for 5000ms; last invalid read %.3fC", temp_c);
        return ESP_FAIL;
    }
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t update_asic_temperature(GlobalState *state, float *temp_c)
{
    esp_err_t err = bitaxe_fan_read_asic_temp_c(temp_c);
    if (err != ESP_OK) {
        if (domain_reboot_recovery_pending()) {
            ESP_LOGW(TAG,
                     "ignoring ASIC temperature read failure while lost-domain auto-reboot is pending: %s",
                     esp_err_to_name(err));
            return err;
        }
        if (xTaskGetTickCount() < g_asic_temp_grace_until) {
            ESP_LOGW(TAG, "ignoring startup ASIC temperature read failure: %s",
                     esp_err_to_name(err));
            return err;
        }
        safety_shutdownf(state, "ASIC temperature read failed: %s (0x%x); value unavailable",
                         esp_err_to_name(err), (unsigned)err);
        return err;
    }

    if (asic_temp_is_no_reading(*temp_c)) {
        return unavailable_asic_temperature(state, *temp_c);
    }

    *temp_c += (float)state->DEVICE_CONFIG.temp_offset;
    if (asic_temp_is_no_reading(*temp_c)) {
        return unavailable_asic_temperature(state, *temp_c);
    }

    if (!isfinite(*temp_c)) {
        if (domain_reboot_recovery_pending()) {
            ESP_LOGW(TAG,
                     "ignoring invalid ASIC temperature while lost-domain auto-reboot is pending");
            return ESP_ERR_INVALID_STATE;
        }
        if (xTaskGetTickCount() < g_asic_temp_grace_until) {
            ESP_LOGW(TAG, "ignoring startup ASIC temperature invalid");
            return ESP_ERR_INVALID_STATE;
        }
        safety_shutdownf(state, "ASIC temperature invalid: %gC (non-finite)", (double)*temp_c);
        return ESP_FAIL;
    }

    g_asic_temp_unavailable_since_us = -1;
    remember_fault_asic_temperature(*temp_c);
    const float asic_temp_shutdown_c =
        (float)m45_config_get()->safety_asic_temp_max_c;
    if (*temp_c >= asic_temp_shutdown_c) {
        if (xTaskGetTickCount() < g_asic_temp_grace_until) {
            ESP_LOGW(TAG, "ignoring startup ASIC temperature %.1f C above %.1f C limit",
                     *temp_c, asic_temp_shutdown_c);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGE(TAG, "ASIC temperature %.1f C reached shutdown limit %.1f C", *temp_c,
                 asic_temp_shutdown_c);
        safety_shutdownf(state, "ASIC temperature %.1fC >= maximum %.1fC",
                         *temp_c, asic_temp_shutdown_c);
        return ESP_FAIL;
    }

    state->POWER_MANAGEMENT_MODULE.chip_temp_avg = *temp_c;
    return ESP_OK;
}

static bool read_asic_temp_for_voltage(GlobalState *state, float *temp_c)
{
    if (temp_c == NULL) {
        return false;
    }

    float raw_temp_c = 0.0f;
    if (bitaxe_fan_read_asic_temp_c(&raw_temp_c) != ESP_OK) {
        *temp_c = 0.0f;
        return false;
    }

    if (asic_temp_is_no_reading(raw_temp_c)) {
        *temp_c = 0.0f;
        return false;
    }

    raw_temp_c += (float)state->DEVICE_CONFIG.temp_offset;
    if (!isfinite(raw_temp_c) || raw_temp_c <= 0.0f ||
        asic_temp_is_no_reading(raw_temp_c)) {
        *temp_c = 0.0f;
        return false;
    }

    remember_fault_asic_temperature(raw_temp_c);
    state->POWER_MANAGEMENT_MODULE.chip_temp_avg = raw_temp_c;
    *temp_c = raw_temp_c;
    return true;
}

static esp_err_t apply_temperature_voltage_compensation(GlobalState *state, float asic_temp_c)
{
    const m45_config_t *config = m45_config_get();
    if (config != NULL && config->overclock_enabled && config->auto_clock_enabled) {
        return ESP_OK;
    }

    const uint16_t target_mv =
        m45_config_effective_asic_voltage_mv_for_temp(config, asic_temp_c);
    if (g_commanded_voltage_mv == 0) {
        g_commanded_voltage_mv = m45_config_effective_asic_voltage_mv(config);
    }

    const int delta_mv = (int)target_mv - (int)g_commanded_voltage_mv;
    if (abs(delta_mv) < ASIC_TEMP_VOLTAGE_UPDATE_DEADBAND_MV) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "temperature voltage compensation %.1f C: %u -> %u mV", asic_temp_c,
             g_commanded_voltage_mv, target_mv);
    return bitaxe_gamma602_set_voltage_mv(state, target_mv);
}

static esp_err_t apply_auto_clock_control(GlobalState *state, float asic_temp_c,
                                          float control_temp_c,
                                          const TPS546_StatusSnapshot *snapshot,
                                          const TPS546_StatusSnapshot *raw_snapshot)
{
    const m45_config_t active_config = *m45_config_get();
    const m45_config_t *config = &active_config;
    const bool active = config->auto_clock_enabled && config->overclock_enabled &&
                        config->fan_override_enabled;
    const uint16_t current_frequency_mhz =
        current_auto_clock_frequency_mhz(state, config);
    const uint16_t current_voltage_mv =
        current_auto_clock_voltage_mv(config, asic_temp_c);
    if (!config->auto_clock_enabled || !active || state->SYSTEM_MODULE.hardware_fault ||
        !isfinite(asic_temp_c) || asic_temp_c <= 0.0f ||
        !isfinite(control_temp_c) || control_temp_c <= 0.0f || snapshot == NULL ||
        raw_snapshot == NULL) {
        memset(&g_auto_clock_voltage, 0, sizeof(g_auto_clock_voltage));
        g_auto_clock_up_dwell_ticks = 0;
        g_auto_clock_down_dwell_ticks = 0;
        update_auto_clock_status(config->auto_clock_enabled, false, current_frequency_mhz,
                                 current_voltage_mv,
                                 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false,
                                 false, false, false,
                                 config->auto_clock_enabled ? "waiting for valid telemetry" : "");
        return ESP_OK;
    }

    // Check raw VIN every monitor pass, before either the startup gate or the
    // five-second thermal-control cadence. The lower of raw/averaged VIN wins.
    const m45_auto_clock_voltage_result_t voltage = m45_auto_clock_voltage_update(
        &g_auto_clock_voltage, raw_snapshot->read_vin, snapshot->read_vin,
        config->safety_input_voltage_min_mv / 1000.0f,
        config->safety_input_voltage_expected_min_mv / 1000.0f,
        (uint64_t)esp_timer_get_time() / 1000ULL);
    char voltage_reason[96] = "";
    if (voltage.limited) {
        g_auto_clock_up_dwell_ticks = 0;
        if (voltage.valid) {
            snprintf(voltage_reason, sizeof(voltage_reason),
                     "TPS VIN %.3fV; guard %.3fV; resume >= %.3fV for 5s",
                     voltage.vin, voltage.guard_v, voltage.resume_v);
        } else {
            snprintf(voltage_reason, sizeof(voltage_reason), "TPS VIN unavailable; holding clock/voltage");
        }
    }
    // Publish promptly even between control ticks; never hide VIN behind an
    // ASIC temperature or current notice.
    portENTER_CRITICAL(&g_auto_clock_status_lock);
    if (voltage.limited || g_auto_clock_status.input_voltage_limited) {
        strlcpy(g_auto_clock_status.hold_reason, voltage_reason,
                sizeof(g_auto_clock_status.hold_reason));
    }
    g_auto_clock_status.input_voltage_limited = voltage.limited;
    portEXIT_CRITICAL(&g_auto_clock_status_lock);

    const char *mining_wait_reason = NULL;
    if ((!auto_clock_startup_gate_ready(&mining_wait_reason) && !voltage.backoff) ||
        !voltage.valid) {
        g_auto_clock_up_dwell_ticks = 0;
        g_auto_clock_down_dwell_ticks = 0;
        update_auto_clock_status(true, false, current_frequency_mhz, current_voltage_mv,
                                 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, voltage.limited, false,
                                 false, false, false,
                                 voltage.limited ? voltage_reason :
                                 (mining_wait_reason != NULL ? mining_wait_reason
                                                              : "waiting for mining telemetry"));
        return ESP_OK;
    }

    const TickType_t now = xTaskGetTickCount();
    if (!voltage.backoff && g_auto_clock_next_tick != 0 &&
        (int32_t)(now - g_auto_clock_next_tick) < 0) {
        return ESP_OK;
    }
    g_auto_clock_next_tick = now + pdMS_TO_TICKS(AUTO_CLOCK_INTERVAL_MS);

    const float v_now = snapshot->read_vout > 0.0f
                            ? snapshot->read_vout
                            : (float)current_voltage_mv / 1000.0f;
    const float i_now = snapshot->read_iout;
    const float p_now = v_now * i_now;
    const float mhz_now = state->POWER_MANAGEMENT_MODULE.actual_frequency > 0.0f
                              ? state->POWER_MANAGEMENT_MODULE.actual_frequency
                              : (float)current_frequency_mhz;
    if (!isfinite(v_now) || !isfinite(i_now) || !isfinite(p_now) || !isfinite(mhz_now) ||
        v_now <= 0.0f || i_now <= 0.0f || p_now <= 0.0f || mhz_now <= 0.0f) {
        g_auto_clock_up_dwell_ticks = 0;
        g_auto_clock_down_dwell_ticks = 0;
        update_auto_clock_status(true, false, current_frequency_mhz, current_voltage_mv,
                                 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, voltage.limited, false,
                                 false, false, false,
                                 voltage.limited ? voltage_reason : "waiting for power telemetry");
        return ESP_OK;
    }

    const float target_temp_c =
        (float)m45_config_effective_auto_clock_target_temp_c(config);
    const float p_target =
        fmaxf(1.0f, p_now + ((target_temp_c - AUTO_CLOCK_TEMP_SAFETY_MARGIN_C -
                              control_temp_c) /
                             AUTO_CLOCK_THERMAL_R_C_PER_W));
    const bool max_power_enabled = config->auto_clock_max_watts_enabled;
    const float max_power_w =
        max_power_enabled
            ? (float)m45_config_effective_auto_clock_max_watts(config)
            : FLT_MAX;
    const uint8_t current_index = auto_clock_index_for_frequency(current_frequency_mhz);
    const float current_ceiling_a = auto_clock_upshift_current_ceiling_a(config);
    const float vr_temp_c = (float)snapshot->read_temp1;
    const float vr_temp_ceiling_c = auto_clock_upshift_vr_temp_ceiling_c(config);
    const bool current_near_limit =
        current_ceiling_a > 0.0f && i_now >= current_ceiling_a;
    const bool vr_temp_near_limit =
        vr_temp_ceiling_c > 0.0f && vr_temp_c >= vr_temp_ceiling_c;
    const bool vin_low_for_upshift = voltage.limited;
    const bool upshift_blocked_by_limits =
        current_near_limit || vr_temp_near_limit || vin_low_for_upshift;
    const bool current_power_over_limit = max_power_enabled && p_now > max_power_w;
    bool input_voltage_limited = voltage.limited;
    bool output_current_limited = false;
    bool vr_temp_limited = false;
    bool power_limited = false;
    bool max_watts_limited = current_power_over_limit;
    bool temperature_limited = false;
    bool have_next_up_candidate = false;
    uint16_t next_up_frequency_mhz = 0;
    float next_up_power_w = 0.0f;
    float next_up_output_current_a = 0.0f;
    uint8_t target_index = 0;
    uint16_t target_base_mv = 0;
    uint16_t target_voltage_mv = 0;
    bool have_target = false;
    bool have_lowest_valid = false;
    uint8_t lowest_valid_index = 0;
    uint8_t highest_valid_index = 0;

    for (uint8_t i = 0; i < auto_clock_preset_count(); ++i) {
        const uint16_t candidate_frequency_mhz = auto_clock_preset_frequency(i);
        uint16_t candidate_base_mv = 0;
        uint16_t candidate_voltage_mv = 0;
        if (!auto_clock_candidate_voltage(config, candidate_frequency_mhz, asic_temp_c,
                                          &candidate_base_mv, &candidate_voltage_mv)) {
            continue;
        }
        if (!have_lowest_valid) {
            lowest_valid_index = i;
            have_lowest_valid = true;
        }
        highest_valid_index = i;

        const float frequency_ratio = (float)candidate_frequency_mhz / mhz_now;
        const float voltage_ratio = ((float)candidate_voltage_mv / 1000.0f) / v_now;
        const float p_est = p_now * frequency_ratio * voltage_ratio * voltage_ratio;
        const bool upshift_candidate = i > current_index;
        const float candidate_v = (float)candidate_voltage_mv / 1000.0f;
        const float i_est = candidate_v > 0.0f ? p_est / candidate_v : 0.0f;
        if (!isfinite(p_est) || !isfinite(i_est) || p_est <= 0.0f ||
            i_est <= 0.0f) {
            continue;
        }
        const bool candidate_current_limited =
            current_ceiling_a > 0.0f && i_est >= current_ceiling_a;
        const float allowed_power =
            upshift_candidate ? p_target * AUTO_CLOCK_UP_POWER_HEADROOM_RATIO : p_target;
        const bool candidate_cooling_limited = p_est > allowed_power;
        const bool candidate_max_watts_limited = max_power_enabled && p_est > max_power_w;
        const bool power_allowed =
            !candidate_cooling_limited && !candidate_max_watts_limited;
        if (upshift_candidate && !have_next_up_candidate) {
            have_next_up_candidate = true;
            next_up_frequency_mhz = candidate_frequency_mhz;
            next_up_power_w = p_est;
            next_up_output_current_a = i_est;
            if (candidate_current_limited || current_near_limit) {
                output_current_limited = true;
            }
            if (vr_temp_near_limit) {
                vr_temp_limited = true;
            }
            if (vin_low_for_upshift) {
                input_voltage_limited = true;
            }
            if (candidate_cooling_limited || candidate_max_watts_limited) {
                power_limited = true;
            }
            if (candidate_max_watts_limited) {
                max_watts_limited = true;
            }
        }
        if (candidate_current_limited ||
            (voltage.limited && candidate_voltage_mv > current_voltage_mv)) {
            continue;
        }
        if (upshift_candidate && upshift_blocked_by_limits) {
            continue;
        }
        if (power_allowed) {
            target_index = i;
            target_base_mv = candidate_base_mv;
            target_voltage_mv = candidate_voltage_mv;
            have_target = true;
        }
    }
    if (!have_lowest_valid) {
        g_auto_clock_up_dwell_ticks = 0;
        g_auto_clock_down_dwell_ticks = 0;
        update_auto_clock_status(true, false, current_frequency_mhz,
                                 current_voltage_mv,
                                 0, p_now, p_target, 0.0f, current_ceiling_a, 0.0f,
                                 voltage.limited, false, false, false, false,
                                 voltage.limited ? voltage_reason : "no preset fits voltage safety limits");
        return ESP_ERR_INVALID_ARG;
    }
    if (!have_target) {
        target_index = lowest_valid_index;
    }
    uint8_t control_current_index = current_index;
    if (control_current_index < lowest_valid_index) {
        control_current_index = lowest_valid_index;
    } else if (control_current_index > highest_valid_index) {
        control_current_index = highest_valid_index;
    }

    uint8_t max_steps_down = AUTO_CLOCK_MAX_STEPS_DOWN;
    bool urgent_downshift = false;
    bool dwell_limited = false;
    const float failsafe_temp_c =
        fmaxf(0.0f, (float)config->safety_asic_temp_max_c - AUTO_CLOCK_FAILSAFE_MARGIN_C);
    if (control_temp_c >= failsafe_temp_c) {
        temperature_limited = true;
        urgent_downshift = true;
        target_index = lowest_valid_index;
        max_steps_down = AUTO_CLOCK_MAX_STEPS_DOWN_HOT;
        bitaxe_fan_force_max_if_allowed(state, "auto clock failsafe");
    } else if (control_temp_c >= target_temp_c + AUTO_CLOCK_HOT_MARGIN_C) {
        temperature_limited = true;
        urgent_downshift = true;
        if (target_index >= control_current_index) {
            target_index = control_current_index > 0 ? control_current_index - 1 : 0;
        }
        max_steps_down = AUTO_CLOCK_MAX_STEPS_DOWN_HOT;
    } else if (control_temp_c >= target_temp_c + AUTO_CLOCK_DOWN_TEMP_HYSTERESIS_C) {
        temperature_limited = true;
        if (target_index >= control_current_index) {
            target_index = control_current_index > 0 ? control_current_index - 1 : 0;
        }
    } else if (control_temp_c >= target_temp_c - AUTO_CLOCK_UP_TEMP_HYSTERESIS_C &&
               target_index > control_current_index) {
        temperature_limited = true;
        target_index = control_current_index;
    }
    if (control_temp_c >= target_temp_c - AUTO_CLOCK_UP_TEMP_HYSTERESIS_C) {
        output_current_limited = false;
        vr_temp_limited = false;
        power_limited = false;
    }
    if (max_watts_limited) {
        power_limited = true;
    }
    if (current_power_over_limit && target_index < control_current_index) {
        urgent_downshift = true;
        max_steps_down = AUTO_CLOCK_MAX_STEPS_DOWN_HOT;
    }
    if (voltage.backoff) {
        // Reduce load now rather than waiting for the thermal dwell. A fresh
        // request is rate-limited to once a second while VIN remains low.
        urgent_downshift = true;
        target_index = lowest_valid_index;
        max_steps_down = AUTO_CLOCK_MAX_STEPS_DOWN;
    }
    if (target_index < lowest_valid_index) {
        target_index = lowest_valid_index;
    }

    uint8_t next_index =
        auto_clock_clamp_toward(control_current_index, target_index, AUTO_CLOCK_MAX_STEPS_UP,
                                max_steps_down);
    if (next_index > control_current_index) {
        g_auto_clock_down_dwell_ticks = 0;
        if (g_auto_clock_up_dwell_ticks < AUTO_CLOCK_UP_DWELL_TICKS) {
            ++g_auto_clock_up_dwell_ticks;
        }
        if (g_auto_clock_up_dwell_ticks < AUTO_CLOCK_UP_DWELL_TICKS) {
            next_index = control_current_index;
            dwell_limited = true;
        }
    } else if (next_index < control_current_index) {
        g_auto_clock_up_dwell_ticks = 0;
        if (!urgent_downshift) {
            if (g_auto_clock_down_dwell_ticks < AUTO_CLOCK_DOWN_DWELL_TICKS) {
                ++g_auto_clock_down_dwell_ticks;
            }
            if (g_auto_clock_down_dwell_ticks < AUTO_CLOCK_DOWN_DWELL_TICKS) {
                next_index = control_current_index;
                dwell_limited = true;
            }
        }
    } else {
        g_auto_clock_up_dwell_ticks = 0;
        g_auto_clock_down_dwell_ticks = 0;
    }
    uint16_t next_frequency_mhz = auto_clock_preset_frequency(next_index);
    uint16_t next_base_mv = 0;
    uint16_t next_voltage_mv = 0;
    bool next_valid = auto_clock_candidate_voltage(config, next_frequency_mhz, asic_temp_c,
                                                    &next_base_mv, &next_voltage_mv);
    // A lower frequency can have a higher preset voltage (notably 525 MHz).
    // Never raise either setting while input voltage is limited. Skip such
    // presets rather than increasing load during a voltage-driven downshift.
    while (voltage.limited && (!next_valid || next_voltage_mv > current_voltage_mv ||
                               next_frequency_mhz > current_frequency_mhz) &&
           next_index > lowest_valid_index) {
        --next_index;
        next_frequency_mhz = auto_clock_preset_frequency(next_index);
        next_valid = auto_clock_candidate_voltage(config, next_frequency_mhz, asic_temp_c,
                                                   &next_base_mv, &next_voltage_mv);
    }
    if (voltage.limited && (!next_valid || next_voltage_mv > current_voltage_mv ||
                            next_frequency_mhz > current_frequency_mhz)) {
        next_frequency_mhz = current_frequency_mhz;
        next_voltage_mv = next_base_mv = current_voltage_mv;
        next_valid = true;
    }
    if (!next_valid) {
        update_auto_clock_status(true, false, current_frequency_mhz,
                                 current_voltage_mv,
                                 next_up_frequency_mhz, p_now, p_target, next_up_power_w,
                                 current_ceiling_a, next_up_output_current_a, voltage.limited, false,
                                 false, false, false,
                                 voltage.limited ? voltage_reason : "next preset outside voltage limits");
        return ESP_ERR_INVALID_ARG;
    }

    auto_clock_candidate_voltage(config, auto_clock_preset_frequency(target_index),
                                 asic_temp_c, &target_base_mv, &target_voltage_mv);
    char hold_reason[96] = "";
    if (voltage.limited) {
        strlcpy(hold_reason, voltage_reason, sizeof(hold_reason));
    } else if (next_frequency_mhz <= current_frequency_mhz) {
        if (dwell_limited) {
            snprintf(hold_reason, sizeof(hold_reason), "waiting for stable trend");
        } else if (temperature_limited) {
            snprintf(hold_reason, sizeof(hold_reason),
                     "temperature %.1f C is near target %.0f C",
                     control_temp_c, target_temp_c);
        } else if (output_current_limited && current_ceiling_a > 0.0f &&
                   next_up_output_current_a > 0.0f) {
            snprintf(hold_reason, sizeof(hold_reason),
                     "IOUT ceiling %.1f A; next %u MHz estimates %.1f A",
                     current_ceiling_a, next_up_frequency_mhz,
                     next_up_output_current_a);
        } else if (vr_temp_limited) {
            snprintf(hold_reason, sizeof(hold_reason),
                     "VR temp %.0f C is near %.0f C ceiling",
                     vr_temp_c, vr_temp_ceiling_c);
        } else if (max_watts_limited && max_power_enabled) {
            if (current_power_over_limit) {
                snprintf(hold_reason, sizeof(hold_reason),
                         "max watts %.0f W; now %.1f W", max_power_w, p_now);
            } else {
                snprintf(hold_reason, sizeof(hold_reason),
                         "max watts %.0f W; next %u MHz estimates %.1f W",
                         max_power_w, next_up_frequency_mhz, next_up_power_w);
            }
        } else if (power_limited && next_up_power_w > 0.0f) {
            snprintf(hold_reason, sizeof(hold_reason),
                     "cooling target %.1f W; next %u MHz estimates %.1f W",
                     p_target, next_up_frequency_mhz, next_up_power_w);
        } else if (have_next_up_candidate) {
            snprintf(hold_reason, sizeof(hold_reason),
                     "next %u MHz does not fit current target", next_up_frequency_mhz);
        } else {
            snprintf(hold_reason, sizeof(hold_reason), "no higher valid preset");
        }
    }
    update_auto_clock_status(true, true, auto_clock_preset_frequency(target_index),
                             target_voltage_mv, next_up_frequency_mhz, p_now, p_target,
                             next_up_power_w, current_ceiling_a,
                             next_up_output_current_a, input_voltage_limited,
                             output_current_limited, vr_temp_limited, power_limited,
                             temperature_limited, hold_reason);

    if (next_frequency_mhz == current_frequency_mhz &&
        next_voltage_mv == current_voltage_mv) {
        return ESP_OK;
    }

    esp_err_t err = auto_clock_apply_preset(state, config, next_frequency_mhz,
                                            next_base_mv, next_voltage_mv);
    if (err != ESP_OK) {
        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "auto clock skipped; ASIC transition in progress");
            return ESP_OK;
        }
        ESP_LOGW(TAG, "auto clock apply failed: %s", esp_err_to_name(err));
    } else {
        g_auto_clock_up_dwell_ticks = 0;
        g_auto_clock_down_dwell_ticks = 0;
    }
    return err;
}

static bool domain_reboot_expected_per_domain_ghs(GlobalState *state,
                                                  const m45_config_t *config,
                                                  const stratum_minimal_stats_t *stats,
                                                  double *expected_per_domain_ghs)
{
    if (state == NULL || config == NULL || stats == NULL ||
        expected_per_domain_ghs == NULL || stats->domain_asic_count == 0 ||
        stats->domain_count == 0) {
        return false;
    }

    const float active_frequency_mhz =
        state->POWER_MANAGEMENT_MODULE.actual_frequency > 0.0f
            ? state->POWER_MANAGEMENT_MODULE.actual_frequency
            : (float)m45_config_effective_asic_frequency_mhz(config);
    if (!isfinite(active_frequency_mhz) || active_frequency_mhz <= 0.0f) {
        return false;
    }

    const double expected_total_ghs =
        (double)active_frequency_mhz *
        (double)state->DEVICE_CONFIG.family.asic.small_core_count *
        (double)stats->domain_asic_count / 1000.0;
    const double domain_slots =
        (double)stats->domain_asic_count * (double)stats->domain_count;
    if (!isfinite(expected_total_ghs) || expected_total_ghs <= 0.0 ||
        domain_slots <= 0.0) {
        return false;
    }

    *expected_per_domain_ghs = expected_total_ghs / domain_slots;
    return isfinite(*expected_per_domain_ghs) && *expected_per_domain_ghs > 0.0;
}

static bool update_domain_reboot_recovery(const stratum_minimal_stats_t *stats,
                                          double threshold_ghs,
                                          TickType_t now,
                                          uint8_t *trigger_asic,
                                          uint8_t *trigger_domain,
                                          double *trigger_hashrate_ghs)
{
    bool should_reboot = false;
    double lowest_ghs = DBL_MAX;
    const uint8_t asic_count = stats->domain_asic_count > STRATUM_HASHRATE_MAX_ASICS
                                   ? STRATUM_HASHRATE_MAX_ASICS
                                   : stats->domain_asic_count;
    const uint8_t domain_count = stats->domain_count > STRATUM_HASH_DOMAIN_COUNT
                                     ? STRATUM_HASH_DOMAIN_COUNT
                                     : stats->domain_count;

    for (uint8_t asic = 0; asic < STRATUM_HASHRATE_MAX_ASICS; ++asic) {
        for (uint8_t domain = 0; domain < STRATUM_HASH_DOMAIN_COUNT; ++domain) {
            if (asic >= asic_count || domain >= domain_count) {
                g_domain_reboot_low_since[asic][domain] = 0;
                continue;
            }

            double hashrate_ghs = stats->domain_hashrates_ghs[asic][domain];
            if (!isfinite(hashrate_ghs) || hashrate_ghs < 0.0) {
                hashrate_ghs = 0.0;
            }
            if (hashrate_ghs >= threshold_ghs) {
                g_domain_reboot_low_since[asic][domain] = 0;
                continue;
            }

            if (g_domain_reboot_low_since[asic][domain] == 0) {
                g_domain_reboot_low_since[asic][domain] = now;
                ESP_LOGW(TAG,
                         "ASIC %u domain %u below %.2f GH/s threshold at %.2f GH/s; waiting %u seconds before auto-reboot",
                         (unsigned)asic, (unsigned)domain, threshold_ghs,
                         hashrate_ghs,
                         (unsigned)(DOMAIN_REBOOT_RECOVERY_MS / 1000));
                continue;
            }

            if ((now - g_domain_reboot_low_since[asic][domain]) >=
                    pdMS_TO_TICKS(DOMAIN_REBOOT_RECOVERY_MS) &&
                hashrate_ghs < lowest_ghs) {
                lowest_ghs = hashrate_ghs;
                *trigger_asic = asic;
                *trigger_domain = domain;
                *trigger_hashrate_ghs = hashrate_ghs;
                should_reboot = true;
            }
        }
    }

    return should_reboot;
}

static esp_err_t apply_domain_reboot_watchdog(GlobalState *state, bool *rebooted)
{
    if (rebooted != NULL) {
        *rebooted = false;
    }

    const m45_config_t *config = m45_config_get();
    const TickType_t now = xTaskGetTickCount();
    if (config == NULL || !config->auto_domain_reboot_enabled || state == NULL ||
        state->SYSTEM_MODULE.hardware_fault || !state->ASIC_initalized ||
        !g_regulator_enabled) {
        reset_domain_reboot_watchdog();
        return ESP_OK;
    }

    if (tick_before(now, g_domain_reboot_cooldown_until) ||
        tick_before(now, g_domain_reboot_grace_until) ||
        tick_before(now, g_domain_reboot_next_tick)) {
        if (tick_before(now, g_domain_reboot_grace_until)) {
            clear_domain_reboot_low_since();
        }
        return ESP_OK;
    }
    g_domain_reboot_next_tick = now + pdMS_TO_TICKS(DOMAIN_REBOOT_CHECK_INTERVAL_MS);

    stratum_minimal_stats_t stats;
    const char *mining_wait_reason = NULL;
    if (!mining_telemetry_ready(&stats, &mining_wait_reason)) {
        ESP_LOGD(TAG, "domain-loss watchdog waiting: %s",
                 mining_wait_reason != NULL ? mining_wait_reason
                                             : "mining telemetry not ready");
        clear_domain_reboot_low_since();
        return ESP_OK;
    }

    if (stats.domain_asic_count == 0 || stats.domain_count == 0) {
        clear_domain_reboot_low_since();
        return ESP_OK;
    }

    double expected_per_domain_ghs = 0.0;
    if (!domain_reboot_expected_per_domain_ghs(state, config, &stats,
                                               &expected_per_domain_ghs)) {
        clear_domain_reboot_low_since();
        return ESP_OK;
    }

    const double threshold_ghs =
        expected_per_domain_ghs * DOMAIN_REBOOT_MIN_EXPECTED_RATIO;
    uint8_t trigger_asic = 0;
    uint8_t trigger_domain = 0;
    double trigger_hashrate_ghs = 0.0;
    if (!update_domain_reboot_recovery(&stats, threshold_ghs, now, &trigger_asic,
                                       &trigger_domain, &trigger_hashrate_ghs)) {
        return ESP_OK;
    }

    ESP_LOGW(TAG,
             "ASIC %u domain %u stayed below %.2f GH/s threshold for %u seconds (%.2f GH/s); auto-rebooting ASIC",
             (unsigned)trigger_asic, (unsigned)trigger_domain, threshold_ghs,
             (unsigned)(DOMAIN_REBOOT_RECOVERY_MS / 1000), trigger_hashrate_ghs);
    clear_domain_reboot_low_since();
    g_domain_reboot_cooldown_until =
        now + pdMS_TO_TICKS(DOMAIN_REBOOT_COOLDOWN_MS);
    g_domain_reboot_next_tick = g_domain_reboot_cooldown_until;
    if (rebooted != NULL) {
        *rebooted = true;
    }

    stratum_minimal_pause_work();
    esp_err_t err = bitaxe_gamma602_set_asic_power(state, false, true);
    vTaskDelay(pdMS_TO_TICKS(DOMAIN_REBOOT_POWER_CYCLE_OFF_MS));
    if (err == ESP_OK && !state->SYSTEM_MODULE.hardware_fault) {
        err = bitaxe_gamma602_set_asic_power(state, true, true);
    }
    stratum_minimal_resume_work();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "domain-loss ASIC auto-reboot failed: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t validate_regulator_startup_snapshot(GlobalState *state,
                                                     const TPS546_StatusSnapshot *snapshot,
                                                     bool *settled,
                                                     float target_vout)
{
    *settled = false;
    bitaxe_gamma602_safety_limits_t limits;
    bitaxe_gamma602_safety_limits(&limits);

    const uint16_t startup_fault_mask = TPS546_STATUS_FAULT_MASK & ~TPS546_STATUS_PGOOD;
    if ((snapshot->valid & (1U << TPS546_FIELD_STATUS)) &&
        (snapshot->status_word & startup_fault_mask)) {
        return fail_regulator_safety(
            state, snapshot,
            "TPS546 startup status fault (limits: VIN %.3f-%.3fV, IOUT warn/fault %.1f/%.1fA, VR %.0fC)",
            limits.input_voltage_min_v, limits.input_voltage_max_v, limits.iout_warn_a, limits.iout_fault_a,
            limits.tps546_temp_max_c);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_STATUS)) &&
        (snapshot->valid & (1U << TPS546_FIELD_INPUT_STATUS)) &&
        (snapshot->status_word & TPS546_STATUS_INPUT) &&
        (snapshot->st_input & TPS546_STATUS_VIN_OVF)) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VIN overvoltage flag; configured limit %.3fV",
            limits.input_voltage_max_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_OPERATION)) &&
        (snapshot->operation & OPERATION_ON) == 0) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 operation is not ON: OP=0x%02x, expected ON bit=0x%02x",
            snapshot->operation, OPERATION_ON);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VIN)) &&
        snapshot->read_vin < limits.input_voltage_min_v) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VIN %.3fV < minimum %.3fV", snapshot->read_vin,
            limits.input_voltage_min_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VIN)) &&
        snapshot->read_vin >= limits.input_voltage_max_v) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VIN %.3fV >= maximum %.3fV", snapshot->read_vin,
            limits.input_voltage_max_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VOUT)) &&
        snapshot->read_vout >= limits.asic_voltage_max_v) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VOUT %.3fV >= maximum %.3fV", snapshot->read_vout,
            limits.asic_voltage_max_v);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_TEMP)) &&
        (float)snapshot->read_temp1 >= limits.tps546_temp_max_c) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 VR temperature %dC >= maximum %.0fC", snapshot->read_temp1,
            limits.tps546_temp_max_c);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_IOUT)) &&
        snapshot->read_iout >= limits.iout_fault_a) {
        return fail_regulator_safety(
            state, snapshot, "TPS546 IOUT %.3fA >= maximum %.3fA", snapshot->read_iout,
            limits.iout_fault_a);
    }

    if ((snapshot->valid & (1U << TPS546_FIELD_VOUT)) &&
        snapshot->read_vout >= limits.asic_voltage_min_v &&
        fabsf(snapshot->read_vout - target_vout) <= TPS546_VOUT_TOLERANCE_VOLTS) {
        *settled = true;
    }

    return ESP_OK;
}

static bool is_tps_communication_error(esp_err_t err)
{
    return err == ESP_FAIL || err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_RESPONSE;
}

static void log_tps_driver_read_event(void *context, const char *reason)
{
    bitaxe_gamma602_log_read_retry((GlobalState *)context, reason);
}

static void log_tps_read_retry(GlobalState *state, const TPS546_StatusSnapshot *snapshot,
                               const char *outcome, unsigned elapsed_ms)
{
    char message[M45_FAULT_MESSAGE_MAX + 1];
    const int prefix = snprintf(message, sizeof(message), "TPS546 read %s after %ums; ",
                                outcome, elapsed_ms);
    TPS546_format_snapshot(snapshot, message + prefix, sizeof(message) - (size_t)prefix);
    format_fault_context(state, message, message, sizeof(message));
    ESP_LOGW(TAG, "%s", message);
    (void)m45_fault_log_record(message);
}

static esp_err_t capture_power_snapshot(GlobalState *state,
                                         TPS546_StatusSnapshot *snapshot, bool startup)
{
    const int64_t started_us = esp_timer_get_time();
    const int64_t retry_window_us = 5000000;
    bool retrying = false;

    while (true) {
        memset(snapshot, 0, sizeof(*snapshot));
        const esp_err_t err = TPS546_snapshot_status(snapshot);
        remember_power_extrema(snapshot);
        // A failed later register read must not hide unsafe values already
        // sampled. Valid bits prevent unread fields being treated as zero.
        bool settled = false;
        const esp_err_t safety_err = startup
            ? validate_regulator_startup_snapshot(state, snapshot, &settled, 0.0f)
            : validate_regulator_safety(state, snapshot, 0.0f, false);
        if (safety_err != ESP_OK) {
            return safety_err;
        }
        const int64_t elapsed_us = esp_timer_get_time() - started_us;
        if (err == ESP_OK) {
            // A latched TPS fault may already have stopped the output before
            // this read. Keep its readback in the cause, not in the retained
            // operating sample used to explain conditions before the trip.
            remember_fault_power_sample(snapshot);
            store_power_snapshot(snapshot);
            if (retrying) {
                log_tps_read_retry(state, snapshot, "recovered", (unsigned)(elapsed_us / 1000));
            }
            return ESP_OK;
        }
        if (!is_tps_communication_error(err)) {
            return fail_regulator_safety(state, snapshot, "TPS546 %s read failed",
                                         startup ? "validation" : "monitor");
        }
        if (!retrying) {
            log_tps_read_retry(state, snapshot, "retrying", (unsigned)(elapsed_us / 1000));
            retrying = true;
        }
        if (elapsed_us >= retry_window_us) {
            return fail_regulator_safety(state, snapshot,
                                         "TPS546 %s read failed for %ums",
                                         startup ? "validation" : "monitor",
                                         (unsigned)(elapsed_us / 1000));
        }
        // Retry reads only. Leave reset/output state unchanged, and do not
        // start another transaction after the five-second observation window.
        const int64_t remaining_ms = (retry_window_us - elapsed_us + 999) / 1000;
        const unsigned delay_ms = remaining_ms < 100 ? (unsigned)remaining_ms : 100;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        if (esp_timer_get_time() - started_us >= retry_window_us) {
            return fail_regulator_safety(state, snapshot,
                                         "TPS546 %s read failed for 5000ms",
                                         startup ? "validation" : "monitor");
        }
    }
}

static esp_err_t wait_for_regulator_after_enable(GlobalState *state, float target_vout)
{
    const TickType_t started = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(TPS546_OUTPUT_SETTLE_TIMEOUT_MS);

    vTaskDelay(pdMS_TO_TICKS(TPS546_OUTPUT_SETTLE_INITIAL_MS));

    while (true) {
        TPS546_StatusSnapshot snapshot;
        esp_err_t err = capture_power_snapshot(state, &snapshot, true);
        if (err != ESP_OK) {
            return err;
        }
        update_power_state(state, &snapshot);

        bool settled = false;
        ESP_RETURN_ON_ERROR(validate_regulator_startup_snapshot(state, &snapshot, &settled,
                                                                target_vout),
                            TAG, "TPS546 startup safety validation failed");
        if (settled) {
            const uint32_t elapsed_ms =
                pdTICKS_TO_MS(xTaskGetTickCount() - started);
            ESP_LOGI(TAG,
                     "TPS546 ok after %lu ms: VIN %.2f V, VOUT %.3f V, IOUT %.2f A, temp %d C, status 0x%04x",
                     (unsigned long)elapsed_ms, snapshot.read_vin, snapshot.read_vout,
                     snapshot.read_iout, snapshot.read_temp1, snapshot.status_word);
            return ESP_OK;
        }

        if ((xTaskGetTickCount() - started) >= timeout_ticks) {
            return fail_regulator_safety(
                state, &snapshot, "TPS546 VOUT %.3fV did not settle to %.3fV +/- %.3fV within %ums",
                snapshot.read_vout, target_vout, TPS546_VOUT_TOLERANCE_VOLTS,
                TPS546_OUTPUT_SETTLE_TIMEOUT_MS);
        }

        vTaskDelay(pdMS_TO_TICKS(TPS546_OUTPUT_SETTLE_POLL_MS));
    }
}

static void power_monitor_task(void *arg)
{
    GlobalState *state = (GlobalState *)arg;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(ASIC_FAN_MONITOR_INTERVAL_MS));
        if (!g_regulator_enabled) {
            continue;
        }

        // Startup and manual power changes own this lock. A snapshot must not
        // straddle a power transition or start concurrent read retry windows.
        if (take_asic_transition_lock(0) != ESP_OK) {
            continue;
        }
        if (!g_regulator_enabled) {
            give_asic_transition_lock();
            continue;
        }

        TPS546_StatusSnapshot snapshot;
        esp_err_t err = capture_power_snapshot(state, &snapshot, false);
        if (err != ESP_OK) {
            give_asic_transition_lock();
            continue;
        }

        if (validate_regulator_safety(
                state, &snapshot,
                (g_commanded_voltage_mv > 0
                     ? g_commanded_voltage_mv
                     : m45_config_effective_asic_voltage_mv(m45_config_get())) /
                    1000.0f,
                false) != ESP_OK) {
            give_asic_transition_lock();
            continue;
        }

        TPS546_StatusSnapshot averaged_snapshot;
        update_power_average(&snapshot, &averaged_snapshot);
        store_power_snapshot(&averaged_snapshot);
        update_power_state(state, &averaged_snapshot);
        give_asic_transition_lock();

        if (state->ASIC_initalized) {
            bool domain_rebooted = false;
            if (apply_domain_reboot_watchdog(state, &domain_rebooted) != ESP_OK) {
                ESP_LOGW(TAG, "domain reboot watchdog update failed");
            }
            if (domain_rebooted || state->SYSTEM_MODULE.hardware_fault ||
                !state->ASIC_initalized || !g_regulator_enabled) {
                continue;
            }

            float asic_temp_c = 0.0f;
            if (update_asic_temperature(state, &asic_temp_c) != ESP_OK) {
                continue;
            }

            const char *mining_wait_reason = NULL;
            const bool mining_started =
                mining_startup_gate_ready(&mining_wait_reason);
            if (mining_started &&
                apply_temperature_voltage_compensation(state, asic_temp_c) != ESP_OK) {
                ESP_LOGW(TAG, "temperature voltage compensation update failed");
                continue;
            }
            const float control_temp_c = bitaxe_fan_control_temp_c(asic_temp_c);
            if (mining_started) {
                if (apply_auto_clock_control(state, asic_temp_c, control_temp_c,
                                             &averaged_snapshot, &snapshot) != ESP_OK) {
                    ESP_LOGW(TAG, "auto clock control update failed");
                }
            } else {
                const m45_config_t *config = m45_config_get();
                if (config != NULL) {
                    const bool auto_clock_enabled = config->auto_clock_enabled;
                    update_auto_clock_status(
                        auto_clock_enabled, false,
                        current_auto_clock_frequency_mhz(state, config),
                        current_auto_clock_voltage_mv(config, asic_temp_c),
                        0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false, false,
                        false, false,
                        auto_clock_enabled
                            ? (mining_wait_reason != NULL ? mining_wait_reason
                                                          : "waiting for mining")
                            : "");
                }
            }
            bitaxe_fan_update_auto(state, asic_temp_c, control_temp_c,
                                   (float)averaged_snapshot.read_temp1);
        }

        if (!state->SYSTEM_MODULE.hardware_fault) {
            state->SYSTEM_MODULE.power_fault = 0;
        }
    }
}

static esp_err_t start_power_monitor(GlobalState *state)
{
    if (g_power_monitor_started) {
        return ESP_OK;
    }

    BaseType_t created =
        xTaskCreate(power_monitor_task, "tps546_mon", TPS546_POWER_MONITOR_STACK_BYTES,
                    state, tskIDLE_PRIORITY + 2, NULL);
    if (created == pdPASS) {
        g_power_monitor_started = true;
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "failed to start TPS546 monitor task");
        return ESP_ERR_NO_MEM;
    }
}

void bitaxe_gamma602_init_state(GlobalState *state)
{
    memset(state, 0, sizeof(*state));
    TPS546_set_read_event_callback(log_tps_driver_read_event, state);

    state->DEVICE_CONFIG = (DeviceConfig) {
        .board_version = "602",
        .family = FAMILY_GAMMA,
        .EMC2101 = true,
        .emc_ideality_factor = 0x24,
        .emc_beta_compensation = 0x00,
        .TPS546 = true,
        .power_consumption_target = 22,
    };
    const m45_config_t *config = m45_config_get();
    state->POWER_MANAGEMENT_MODULE.frequency_value =
        (float)m45_config_effective_asic_frequency_mhz(config);
    state->POWER_MANAGEMENT_MODULE.actual_frequency = 0.0f;
    state->POWER_MANAGEMENT_MODULE.core_voltage =
        m45_config_effective_asic_voltage_mv(config) / 1000.0f;
    state->pool_difficulty = m45_config_effective_pool_difficulty(
        config, state->DEVICE_CONFIG.family.asic.small_core_count,
        state->DEVICE_CONFIG.family.asic_count);
    state->version_mask = STRATUM_DEFAULT_VERSION_MASK;
    strlcpy(state->SYSTEM_MODULE.pool_user, config->pool_user,
            sizeof(state->SYSTEM_MODULE.pool_user));
    strlcpy(state->SYSTEM_MODULE.pool_pass, config->pool_pass,
            sizeof(state->SYSTEM_MODULE.pool_pass));
    strlcpy(state->SYSTEM_MODULE.pool_url, config->pool_host,
            sizeof(state->SYSTEM_MODULE.pool_url));
    state->SYSTEM_MODULE.pool_port = config->pool_port;
    state->send_uid = 10;
    state->stratum_mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    pthread_mutex_init(&state->valid_jobs_lock, NULL);
    const esp_err_t lock_err = ensure_asic_transition_lock();
    if (lock_err != ESP_OK) {
        ESP_LOGE(TAG, "failed to create ASIC transition lock: %s",
                 esp_err_to_name(lock_err));
    }
    reset_mining_telemetry_gate();

    state->ASIC_TASK_MODULE.active_jobs = calloc(ASIC_JOB_SLOTS, sizeof(bm_job *));
    state->valid_jobs = calloc(ASIC_JOB_SLOTS, sizeof(uint8_t));
    if (state->ASIC_TASK_MODULE.active_jobs == NULL || state->valid_jobs == NULL) {
        ESP_LOGE(TAG, "failed to allocate ASIC job tables");
        set_hw_status("job table alloc failed");
    }
}

esp_err_t bitaxe_gamma602_prepare_io(void)
{
    if (!g_i2c_ready) {
        ESP_RETURN_ON_ERROR(i2c_bitaxe_init(), TAG, "I2C init failed");
        g_i2c_ready = true;
    }
    ESP_RETURN_ON_ERROR(asic_reset_level(0), TAG, "ASIC reset hold failed");
    set_hw_status("reset held");
    return ESP_OK;
}

esp_err_t bitaxe_gamma602_boot_fan_max(void)
{
    set_hw_status("fan max");
    ESP_LOGI(TAG, "setting EMC2101 fan to max before NVS, display, network, and TPS546");
    return bitaxe_fan_boot_max();
}

esp_err_t bitaxe_gamma602_start_fan(GlobalState *state)
{
    set_hw_status("fan init");
    ESP_LOGI(TAG, "initializing EMC2101 fan before display, network, and TPS546");
    return bitaxe_fan_init(state);
}

static esp_err_t initialize_hardware_unlocked(GlobalState *state)
{
    if (state->ASIC_TASK_MODULE.active_jobs == NULL || state->valid_jobs == NULL) {
        return shutdown_and_return(state, "ASIC job tables missing", ESP_ERR_NO_MEM);
    }

    esp_err_t ret = bitaxe_gamma602_start_fan(state);
    if (ret != ESP_OK) {
        return shutdown_and_return(state, "ASIC fan init failed", ret);
    }
    clear_power_snapshot();
    reset_power_average_window();

    set_hw_status("regulator init");
    ESP_LOGI(TAG, "initializing TPS546 for Gamma 602");
    ret = TPS546_init(gamma_tps546_config());
    if (ret != ESP_OK) {
        return shutdown_and_return(state, "TPS546 init failed", ret);
    }
    g_tps546_ready = true;

    const m45_config_t *config = m45_config_get();
    float asic_temp_c = 0.0f;
    const bool have_startup_temp = read_asic_temp_for_voltage(state, &asic_temp_c);
    const uint16_t voltage_mv =
        m45_config_effective_asic_voltage_mv_for_temp(config, asic_temp_c);
    const int16_t compensation_mv =
        m45_config_asic_voltage_temp_compensation_mv(config, asic_temp_c);
    const float volts = voltage_mv / 1000.0f;
    if (compensation_mv != 0) {
        ESP_LOGI(TAG, "ASIC temp %.1f C applies %+d mV voltage compensation", asic_temp_c,
                 compensation_mv);
    } else if (config->overclock_enabled && config->asic_voltage_temp_compensation_enabled &&
               !have_startup_temp) {
        ESP_LOGW(TAG, "ASIC temp unavailable; starting without voltage compensation");
    }
    ESP_LOGI(TAG, "setting BM1370 core voltage to %.3f V", volts);
    ret = TPS546_set_vout(volts);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_STATE) {
            TPS546_StatusSnapshot snapshot;
            (void)TPS546_snapshot_status(&snapshot);
            return fail_regulator_safety(state, &snapshot,
                                         "TPS546 output command did not latch ON");
        }
        return shutdown_and_return(state, "TPS546 set voltage failed", ret);
    }
    g_commanded_voltage_mv = voltage_mv;
    g_regulator_enabled = true;
    ret = wait_for_regulator_after_enable(state, volts);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = start_power_monitor(state);
    if (ret != ESP_OK) {
        return shutdown_and_return(state, "TPS546 monitor start failed", ret);
    }

    set_hw_status("asic reset");
    ret = asic_reset_pulse();
    if (ret != ESP_OK) {
        return shutdown_and_return(state, "ASIC reset failed", ret);
    }

    set_hw_status("serial init");
    ret = SERIAL_init();
    if (ret != ESP_OK) {
        return shutdown_and_return(state, "ASIC serial init failed", ret);
    }
    // Discard nonce/partial response bytes left over from the previous run
    // before sending the chip-ID request at the startup baud.
    SERIAL_clear_buffer();

    set_hw_status("asic detect");
    g_chip_count = BM1370_init(state);
    if (g_chip_count == 0) {
        set_hw_status("no BM1370");
        ESP_LOGE(TAG, "no BM1370 chips detected");
        return shutdown_and_return(state, "no BM1370 detected", ESP_FAIL);
    }

    ret = SERIAL_set_baud(BM1370_set_max_baud());
    if (ret != ESP_OK) {
        return shutdown_and_return(state, "ASIC baud change failed", ret);
    }
    SERIAL_clear_buffer();
    // The power monitor skips transitions while this task holds the lock.
    // Recheck the regulator after the handshake before exposing a ready ASIC.
    ret = wait_for_regulator_after_enable(state, volts);
    if (ret != ESP_OK) {
        return ret;
    }
    // A known-valid hot ASIC must not be exposed as ready while its startup
    // sensor grace is active. Missing/sentinel readings retain their existing
    // startup handling in update_asic_temperature().
    float ready_temp_c = 0.0f;
    if (read_asic_temp_for_voltage(state, &ready_temp_c) &&
        isfinite(ready_temp_c) && !asic_temp_is_no_reading(ready_temp_c) &&
        ready_temp_c >= (float)config->safety_asic_temp_max_c) {
        safety_shutdownf(state, "ASIC temperature %.1fC >= maximum %.1fC during startup",
                         ready_temp_c, (float)config->safety_asic_temp_max_c);
        return ESP_FAIL;
    }
    if (state->SYSTEM_MODULE.hardware_fault || !g_regulator_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    state->ASIC_initalized = true;
    g_asic_temp_grace_until =
        xTaskGetTickCount() + pdMS_TO_TICKS(ASIC_TEMP_STARTUP_GRACE_MS);
    g_domain_reboot_grace_until =
        xTaskGetTickCount() + pdMS_TO_TICKS(DOMAIN_REBOOT_STARTUP_GRACE_MS);
    set_hw_status("ready");

    ESP_LOGI(TAG, "Gamma 602 ASIC ready: %u chip(s), %.0f MHz, %d mV",
             g_chip_count, state->POWER_MANAGEMENT_MODULE.actual_frequency,
             g_commanded_voltage_mv);
    return ESP_OK;
}

static esp_err_t bitaxe_gamma602_start_hardware_unlocked(GlobalState *state)
{
    g_asic_temp_unavailable_since_us = -1;
    if (state->SYSTEM_MODULE.hardware_fault) {
        return ESP_ERR_INVALID_STATE;
    }
    const bool was_paused = stratum_minimal_work_paused();
    stratum_minimal_pause_work();
    state->ASIC_initalized = false;

    // Startup is requested explicitly by boot/manual power-on. Hold reset
    // before draining the old mining receive so it cannot steal chip-ID data.
    esp_err_t err = asic_reset_level(0);
    if (err != ESP_OK) {
        err = shutdown_and_return(state, "ASIC reset hold failed", err);
    } else if (!stratum_minimal_wait_for_asic_rx_idle(ASIC_RX_DRAIN_TIMEOUT_MS)) {
        err = shutdown_and_return(state, "ASIC UART receive did not become idle before startup",
                                  ESP_ERR_TIMEOUT);
    } else {
        // Exactly one initialization. Communication grace retries reads in the
        // drivers; it never repeats a reset, regulator enable, or ASIC startup.
        err = initialize_hardware_unlocked(state);
    }
    if (err == ESP_OK) {
        state->SYSTEM_MODULE.hardware_fault_msg[0] = '\0';
    } else if (!state->SYSTEM_MODULE.hardware_fault) {
        set_hw_status("asic off");
    }
    if (!was_paused) {
        stratum_minimal_resume_work();
    }
    return err;
}

esp_err_t bitaxe_gamma602_start_hardware(GlobalState *state)
{
    ESP_RETURN_ON_ERROR(take_asic_transition_lock(portMAX_DELAY), TAG,
                        "ASIC transition lock failed");
    reset_mining_telemetry_gate();
    const esp_err_t err = bitaxe_gamma602_start_hardware_unlocked(state);
    give_asic_transition_lock();
    return err;
}

bool bitaxe_gamma602_asic_power_enabled(void)
{
    return g_regulator_enabled;
}

esp_err_t bitaxe_gamma602_set_asic_power(GlobalState *state, bool enabled, bool manage_fan)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(take_asic_transition_lock(portMAX_DELAY), TAG,
                        "ASIC transition lock failed");
    reset_mining_telemetry_gate();
    esp_err_t err = ESP_OK;

    if (enabled) {
        if (state->SYSTEM_MODULE.hardware_fault) {
            err = ESP_ERR_INVALID_STATE;
            goto out;
        }
        if (state->ASIC_initalized && g_regulator_enabled) {
            goto out;
        }
        const m45_config_t *config = m45_config_get();
        state->POWER_MANAGEMENT_MODULE.frequency_value =
            (float)m45_config_effective_asic_frequency_mhz(config);
        state->POWER_MANAGEMENT_MODULE.actual_frequency = 0.0f;
        TickType_t off_tick = 0;
        if (consume_asic_power_off_tick(&off_tick)) {
            const TickType_t minimum_off_ticks = pdMS_TO_TICKS(ASIC_POWER_OFF_MIN_MS);
            const TickType_t elapsed = xTaskGetTickCount() - off_tick;
            if (elapsed < minimum_off_ticks) {
                vTaskDelay(minimum_off_ticks - elapsed);
            }
        }
        if (manage_fan) {
            err = bitaxe_fan_start_for_asic(state);
            if (err != ESP_OK) {
                err = shutdown_and_return(state, "ASIC fan start failed", err);
                set_hw_status("asic off");
                goto out;
            }
        }
        reset_domain_reboot_recovery();
        g_asic_temp_grace_until =
            xTaskGetTickCount() + pdMS_TO_TICKS(ASIC_TEMP_STARTUP_GRACE_MS);
        bitaxe_gamma602_clear_jobs(state);
        stratum_minimal_reset_hashrate();
        err = bitaxe_gamma602_start_hardware_unlocked(state);
        if (err != ESP_OK && !state->SYSTEM_MODULE.hardware_fault) {
            set_hw_status("asic off");
        }
        goto out;
    }

    ESP_LOGI(TAG, "manual ASIC power off requested");
    remember_fault_operating_settings(state);
    set_hw_status("asic off");
    reset_domain_reboot_recovery();
    g_regulator_enabled = false;
    g_commanded_voltage_mv = 0;
    g_chip_count = 0;
    state->ASIC_initalized = false;
    state->POWER_MANAGEMENT_MODULE.actual_frequency = 0.0f;
    if (!state->SYSTEM_MODULE.hardware_fault) {
        state->SYSTEM_MODULE.power_fault = 0;
    }
    bitaxe_gamma602_clear_jobs(state);
    stratum_minimal_reset_hashrate();
    clear_power_snapshot();
    state->POWER_MANAGEMENT_MODULE.power = 0.0f;
    state->POWER_MANAGEMENT_MODULE.current = 0.0f;
    state->POWER_MANAGEMENT_MODULE.core_voltage = 0.0f;
    state->POWER_MANAGEMENT_MODULE.chip_temp_avg = 0.0f;

    err = asic_reset_level(0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to hold ASIC reset low during manual power off: %s",
                 esp_err_to_name(err));
    }

    if (g_tps546_ready) {
        const esp_err_t vout_err = TPS546_set_vout(0.0f);
        if (vout_err != ESP_OK) {
            ESP_LOGW(TAG, "failed to turn TPS546 output off manually: %s",
                     esp_err_to_name(vout_err));
            if (err == ESP_OK) {
                err = vout_err;
            }
        }
    }
    record_asic_power_off();

    if (manage_fan) {
        const esp_err_t fan_err = bitaxe_fan_stop_for_asic(state);
        if (fan_err != ESP_OK && err == ESP_OK) {
            err = fan_err;
        }
    }

out:
    give_asic_transition_lock();
    return err;
}

static esp_err_t bitaxe_gamma602_set_frequency_mhz_unlocked(GlobalState *state,
                                                            uint16_t frequency_mhz)
{
    if (state == NULL || frequency_mhz < M45_ASIC_FREQUENCY_MIN_MHZ ||
        frequency_mhz > M45_ASIC_FREQUENCY_MAX_MHZ) {
        return ESP_ERR_INVALID_ARG;
    }

    state->POWER_MANAGEMENT_MODULE.frequency_value = (float)frequency_mhz;
    if (!state->ASIC_initalized) {
        return ESP_OK;
    }

    set_hw_status("clock update");
    SERIAL_clear_buffer();
    bitaxe_gamma602_clear_jobs(state);
    do_frequency_transition(state, BM1370_send_hash_frequency);
    BM1370_set_nonce_space(BM1370_NONCE_SPACE_PERCENT,
                           state->POWER_MANAGEMENT_MODULE.actual_frequency,
                           bitaxe_gamma602_chip_count(),
                           state->DEVICE_CONFIG.family.asic.core_count);
    vTaskDelay(pdMS_TO_TICKS(ASIC_FREQUENCY_SETTLE_MS));
    SERIAL_clear_buffer();
    bitaxe_gamma602_clear_jobs(state);
    set_hw_status("ready");
    return ESP_OK;
}

esp_err_t bitaxe_gamma602_set_frequency_mhz(GlobalState *state, uint16_t frequency_mhz)
{
    ESP_RETURN_ON_ERROR(take_asic_transition_lock(portMAX_DELAY), TAG,
                        "ASIC transition lock failed");
    const esp_err_t err = bitaxe_gamma602_set_frequency_mhz_unlocked(state, frequency_mhz);
    give_asic_transition_lock();
    return err;
}

static esp_err_t bitaxe_gamma602_set_voltage_mv_for_config_unlocked(
    GlobalState *state, uint16_t voltage_mv, const m45_config_t *config)
{
    const m45_config_t *active = config != NULL ? config : m45_config_get();
    if (state == NULL || voltage_mv < 500 || voltage_mv > 1370 ||
        voltage_mv < active->safety_asic_voltage_min_mv ||
        voltage_mv >= active->safety_asic_voltage_max_mv) {
        return ESP_ERR_INVALID_ARG;
    }

    const float volts = voltage_mv / 1000.0f;
    state->POWER_MANAGEMENT_MODULE.core_voltage = volts;
    if (!g_tps546_ready || !g_regulator_enabled) {
        g_commanded_voltage_mv = voltage_mv;
        return ESP_OK;
    }

    set_hw_status("voltage update");
    esp_err_t ret = TPS546_set_vout(volts);
    if (ret == ESP_ERR_INVALID_STATE) {
        TPS546_StatusSnapshot snapshot;
        (void)TPS546_snapshot_status(&snapshot);
        return fail_regulator_safety(state, &snapshot,
                                     "TPS546 output command did not latch ON");
    }
    if (ret != ESP_OK) {
        (void)shutdown_and_return(state, "TPS546 voltage update failed", ret);
        set_hw_status("asic off");
        return ret;
    }
    ret = wait_for_regulator_after_enable(state, volts);
    if (ret == ESP_OK) {
        g_commanded_voltage_mv = voltage_mv;
        set_hw_status("ready");
    }
    return ret;
}

esp_err_t bitaxe_gamma602_set_voltage_mv_for_config(GlobalState *state, uint16_t voltage_mv,
                                                    const m45_config_t *config)
{
    ESP_RETURN_ON_ERROR(take_asic_transition_lock(portMAX_DELAY), TAG,
                        "ASIC transition lock failed");
    const esp_err_t err =
        bitaxe_gamma602_set_voltage_mv_for_config_unlocked(state, voltage_mv, config);
    give_asic_transition_lock();
    return err;
}

esp_err_t bitaxe_gamma602_set_voltage_mv(GlobalState *state, uint16_t voltage_mv)
{
    return bitaxe_gamma602_set_voltage_mv_for_config(state, voltage_mv, m45_config_get());
}

esp_err_t bitaxe_gamma602_apply_safety_limits(const m45_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_tps546_ready) {
        return ESP_OK;
    }

    set_hw_status("limit update");
    const esp_err_t ret = TPS546_apply_limits(gamma_tps546_config_from(config));
    if (ret == ESP_OK) {
        set_hw_status("ready");
    }
    return ret;
}

void bitaxe_gamma602_clear_jobs(GlobalState *state)
{
    if (state->ASIC_TASK_MODULE.active_jobs == NULL || state->valid_jobs == NULL) {
        return;
    }

    pthread_mutex_lock(&state->valid_jobs_lock);
    for (int i = 0; i < ASIC_JOB_SLOTS; ++i) {
        state->valid_jobs[i] = 0;
        if (state->ASIC_TASK_MODULE.active_jobs[i] != NULL) {
            free_bm_job(state->ASIC_TASK_MODULE.active_jobs[i]);
            state->ASIC_TASK_MODULE.active_jobs[i] = NULL;
        }
    }
    pthread_mutex_unlock(&state->valid_jobs_lock);
}

void bitaxe_gamma602_clear_pool_jobs(GlobalState *state, uint8_t pool_id)
{
    if (state->ASIC_TASK_MODULE.active_jobs == NULL || state->valid_jobs == NULL) {
        return;
    }

    pthread_mutex_lock(&state->valid_jobs_lock);
    for (int i = 0; i < ASIC_JOB_SLOTS; ++i) {
        bm_job *job = state->ASIC_TASK_MODULE.active_jobs[i];
        if (job == NULL || job->pool_id != pool_id) {
            continue;
        }
        state->valid_jobs[i] = 0;
        free_bm_job(job);
        state->ASIC_TASK_MODULE.active_jobs[i] = NULL;
    }
    pthread_mutex_unlock(&state->valid_jobs_lock);
}

uint8_t bitaxe_gamma602_chip_count(void)
{
    return g_chip_count;
}

const char *bitaxe_gamma602_status(void)
{
    return g_hw_status;
}

const char *bitaxe_gamma602_tps_model(void)
{
    return TPS546_model();
}

void bitaxe_gamma602_telemetry_extrema(m45_telemetry_extrema_t *extrema)
{
    if (extrema == NULL) {
        return;
    }
    portENTER_CRITICAL(&g_power_snapshot_lock);
    *extrema = g_telemetry_extrema;
    portEXIT_CRITICAL(&g_power_snapshot_lock);
}

bool bitaxe_gamma602_power_snapshot(bitaxe_gamma602_power_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_power_snapshot_lock);
    *snapshot = g_power_snapshot;
    portEXIT_CRITICAL(&g_power_snapshot_lock);

    return snapshot->valid;
}

void bitaxe_gamma602_safety_limits(bitaxe_gamma602_safety_limits_t *limits)
{
    if (limits == NULL) {
        return;
    }

    const uint16_t target_voltage_mv =
        g_commanded_voltage_mv > 0 ? g_commanded_voltage_mv
                                   : m45_config_effective_asic_voltage_mv(m45_config_get());
    const m45_config_t *active = m45_config_get();
    const float target_vout = target_voltage_mv / 1000.0f;
    const float asic_voltage_min_v = active->safety_asic_voltage_min_mv / 1000.0f;
    const float asic_voltage_max_v = active->safety_asic_voltage_max_mv / 1000.0f;
    const float expected_vout_min =
        fmaxf(asic_voltage_min_v, target_vout - TPS546_VOUT_TOLERANCE_VOLTS);
    const float expected_vout_max =
        fminf(asic_voltage_max_v, target_vout + TPS546_VOUT_TOLERANCE_VOLTS);
    const float iout_warn_a = active->safety_iout_warn_deciamps / 10.0f;
    const float iout_fault_a = active->safety_iout_fault_deciamps / 10.0f;

    *limits = (bitaxe_gamma602_safety_limits_t){
        .input_voltage_min_v = active->safety_input_voltage_min_mv / 1000.0f,
        .input_voltage_expected_min_v =
            active->safety_input_voltage_expected_min_mv / 1000.0f,
        .input_voltage_expected_max_v =
            active->safety_input_voltage_expected_max_mv / 1000.0f,
        .input_voltage_max_v = active->safety_input_voltage_max_mv / 1000.0f,
        .asic_voltage_min_v = asic_voltage_min_v,
        .asic_voltage_expected_min_v = expected_vout_min,
        .asic_voltage_expected_max_v = expected_vout_max,
        .asic_voltage_max_v = asic_voltage_max_v,
        .asic_voltage_target_v = target_vout,
        .asic_temp_expected_max_c = (float)active->safety_asic_temp_expected_max_c,
        .asic_temp_max_c = (float)active->safety_asic_temp_max_c,
        .tps546_temp_expected_max_c = (float)active->safety_tps546_temp_expected_max_c,
        .tps546_temp_max_c = (float)active->safety_tps546_temp_max_c,
        .iout_warn_a = iout_warn_a,
        .iout_fault_a = iout_fault_a,
        .power_warn_w = target_vout * iout_warn_a,
        .power_fault_w = target_vout * iout_fault_a,
        .fan_expected_percent = 100.0f,
    };
}

void bitaxe_gamma602_auto_clock_status(bitaxe_gamma602_auto_clock_status_t *status)
{
    if (status == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_auto_clock_status_lock);
    *status = g_auto_clock_status;
    portEXIT_CRITICAL(&g_auto_clock_status_lock);
}
