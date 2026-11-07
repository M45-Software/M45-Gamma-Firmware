#include <assert.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tps546.h"
#include "m45_fault_context.h"
#include "m45_telemetry_extrema.h"
#define ESP_ERR_INVALID_RESPONSE 0x108
#define ESP_ERR_INVALID_STATE 0x103
#define M45_FAULT_MESSAGE_MAX 1023
#define pdMS_TO_TICKS(ms) (ms)
#define ESP_LOGW(tag, ...) do { (void)(tag); if (0) printf(__VA_ARGS__); } while (0)
#define ESP_LOGE(tag, ...) ESP_LOGW(tag, __VA_ARGS__)
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
static int g_power_snapshot_lock;
static m45_telemetry_extrema_t g_telemetry_extrema;
static const char *TAG = "test";
typedef struct {
    struct { bool hardware_fault; char hardware_fault_msg[1024]; } SYSTEM_MODULE;
    struct { float core_voltage; } POWER_MANAGEMENT_MODULE;
} GlobalState;
typedef struct { uint16_t safety_asic_voltage_min_mv, safety_asic_voltage_max_mv; } m45_config_t;
typedef struct {
    float input_voltage_min_v, input_voltage_max_v, asic_voltage_min_v, asic_voltage_max_v;
    float iout_warn_a, iout_fault_a, tps546_temp_max_c;
} bitaxe_gamma602_safety_limits_t;
static const m45_config_t config = {700, 1400};
static bool g_tps546_ready = true, g_regulator_enabled = true;
static uint16_t g_commanded_voltage_mv;
static unsigned read_calls, write_calls, records, delay_calls, remembered, stored;
static unsigned read_duration_ms;
static int failures_left;
static int64_t now_us;
static esp_err_t write_result;
static TPS546_StatusSnapshot sample;
static m45_fault_context_t fault_context;
static char last_reason[1024], hardware_status[64];
static const m45_config_t *m45_config_get(void) { return &config; }
static void set_hw_status(const char *s) { snprintf(hardware_status, sizeof(hardware_status), "%s", s); }
const char *esp_err_to_name(esp_err_t e) { return e == ESP_ERR_TIMEOUT ? "ESP_ERR_TIMEOUT" : "ESP_FAIL"; }
static int64_t esp_timer_get_time(void) { return now_us; }
static void vTaskDelay(unsigned ticks) {
    assert(g_regulator_enabled); // Retrying a read must not turn off output.
    assert(!write_calls || write_calls == 1); // A runtime tune writes once.
    delay_calls++;
    now_us += (int64_t)ticks * 1000;
}
static void remember_fault_power_sample(const TPS546_StatusSnapshot *s) {
    remembered++;
    m45_fault_context_remember_power(&fault_context, (uint64_t)now_us / 1000,
        s->read_vin, s->read_vout, s->read_iout, (float)s->read_temp1);
}
static void store_power_snapshot(const TPS546_StatusSnapshot *s) { (void)s; stored++; }
static void format_fault_context(GlobalState *s, const char *reason, char *dst, size_t size) {
    (void)s;
    m45_fault_context_format(&fault_context, (uint64_t)now_us / 1000, reason, dst, size);
}
static esp_err_t m45_fault_log_record(const char *message) {
    records++; snprintf(last_reason, sizeof(last_reason), "%s", message); return ESP_OK;
}
static void bitaxe_gamma602_safety_limits(bitaxe_gamma602_safety_limits_t *l) {
    *l = (bitaxe_gamma602_safety_limits_t){4.8f, 5.5f, .7f, 1.4f, 30, 33, 110};
}
static void flag_safety_fault(GlobalState *s, const char *reason) {
    s->SYSTEM_MODULE.hardware_fault = true;
    snprintf(s->SYSTEM_MODULE.hardware_fault_msg, sizeof(s->SYSTEM_MODULE.hardware_fault_msg), "%s", reason);
}
static esp_err_t shutdown_regulator(GlobalState *s, const char *reason) {
    (void)s; (void)reason; g_regulator_enabled = false;
    // Simulate cleared/live telemetry after shutdown, before NVS persistence.
    m45_fault_context_remember_power(&fault_context, (uint64_t)now_us / 1000, 5.2f, 0, 0, 25);
    return ESP_OK;
}
static void persist_telemetry_extrema(void) { assert(!g_regulator_enabled); }
static esp_err_t shutdown_and_return(GlobalState *s, const char *reason, esp_err_t err) {
    (void)s; snprintf(last_reason, sizeof(last_reason), "%s", reason); g_regulator_enabled = false; return err;
}
esp_err_t TPS546_snapshot_status(TPS546_StatusSnapshot *out) {
    read_calls++; now_us += (int64_t)read_duration_ms * 1000;
    *out = sample;
    if (failures_left != 0) {
        if (failures_left > 0) failures_left--;
        return out->read_error;
    }
    *out = (TPS546_StatusSnapshot){.valid=TPS546_SNAPSHOT_ALL_VALID,.operation=OPERATION_ON,
        .read_vin=5.1f,.read_vout=1.15f,.read_iout=20,.read_temp1=70};
    return ESP_OK;
}
esp_err_t TPS546_set_vout(float volts) { (void)volts; write_calls++; return write_result; }
static esp_err_t wait_for_regulator_after_enable(GlobalState *s, float volts);
/* POWER_VALIDATION_UNDER_TEST */
static esp_err_t wait_for_regulator_after_enable(GlobalState *s, float volts) {
    (void)volts; TPS546_StatusSnapshot out; return capture_power_snapshot(s, &out, true);
}
static void reset(void) {
    g_regulator_enabled = true; write_calls=read_calls=records=delay_calls=remembered=stored=0;
    failures_left=0; now_us=0; read_duration_ms=0; write_result=ESP_OK;
    sample=(TPS546_StatusSnapshot){.read_error=ESP_ERR_TIMEOUT,.failed_command=0x79};
    fault_context=(m45_fault_context_t){0};
    g_telemetry_extrema=(m45_telemetry_extrema_t){0};
    m45_fault_context_remember_settings(&fault_context, 0, 700, 700, 1200, 1200);
    m45_fault_context_remember_asic_temp(&fault_context, 0, 55);
    m45_fault_context_remember_power(&fault_context, 0, 5.1f, 1.2f, 29, 80);
    hardware_status[0]=last_reason[0]='\0';
}
static void expect_safety(TPS546_StatusSnapshot unsafe, bool startup, const char *reason) {
    reset(); sample=unsafe; failures_left=-1; GlobalState state={0}; TPS546_StatusSnapshot out;
    read_duration_ms=250;
    assert(capture_power_snapshot(&state,&out,startup)!=ESP_OK);
    assert(state.SYSTEM_MODULE.hardware_fault && !g_regulator_enabled);
    assert(read_calls==1 && delay_calls==0 && write_calls==0 && records==1 && remembered==0);
    assert(strstr(last_reason,reason));
    assert(!strcmp(state.SYSTEM_MODULE.hardware_fault_msg, last_reason));
}
int main(void) {
    GlobalState state={0}; TPS546_StatusSnapshot out;
    reset(); failures_left=1;
    assert(capture_power_snapshot(&state,&out,false)==ESP_OK);
    assert(read_calls==2 && delay_calls==1 && records==2 && stored==1 && remembered==1);
    assert(g_regulator_enabled && !state.SYSTEM_MODULE.hardware_fault && write_calls==0);
    assert(strstr(last_reason,"recovered") && strstr(last_reason,"actual=700MHz"));
    reset(); failures_left=-1;
    assert(capture_power_snapshot(&state,&out,false)!=ESP_OK);
    assert(now_us==5000000 && read_calls==50 && records==2 && write_calls==0);
    assert(state.SYSTEM_MODULE.hardware_fault && strstr(last_reason,"5000ms"));
    state=(GlobalState){0}; reset(); failures_left=-1; read_duration_ms=1500;
    assert(capture_power_snapshot(&state,&out,true)!=ESP_OK);
    assert(now_us==6300000 && read_calls==4); // Already-started I2C call can finish after deadline.
    TPS546_StatusSnapshot unsafe={.read_error=ESP_ERR_TIMEOUT,.valid=1U<<TPS546_FIELD_STATUS,
        .status_word=TPS546_STATUS_IOUT_OC};
    expect_safety(unsafe,false,"status fault"); expect_safety(unsafe,true,"status fault");
    unsafe=(TPS546_StatusSnapshot){.read_error=ESP_ERR_TIMEOUT,.valid=1U<<TPS546_FIELD_INPUT_STATUS | 1U<<TPS546_FIELD_STATUS,
        .status_word=TPS546_STATUS_INPUT,.st_input=TPS546_STATUS_VIN_OVF};
    expect_safety(unsafe,false,"VIN overvoltage");
    unsafe=(TPS546_StatusSnapshot){.read_error=ESP_ERR_TIMEOUT,.valid=1U<<TPS546_FIELD_VIN,.read_vin=4.7f};
    expect_safety(unsafe,false,"VIN 4.700V <"); expect_safety(unsafe,true,"VIN 4.700V <");
    unsafe.read_vin=5.5f; expect_safety(unsafe,false,"VIN 5.500V >=");
    unsafe=(TPS546_StatusSnapshot){.read_error=ESP_ERR_TIMEOUT,.valid=1U<<TPS546_FIELD_VOUT,.read_vout=1.4f};
    expect_safety(unsafe,false,"VOUT 1.400V >="); expect_safety(unsafe,true,"VOUT 1.400V >=");
    unsafe=(TPS546_StatusSnapshot){.read_error=ESP_ERR_TIMEOUT,.valid=1U<<TPS546_FIELD_IOUT,.read_iout=33};
    expect_safety(unsafe,false,"IOUT 33.000A >=");
    unsafe=(TPS546_StatusSnapshot){.read_error=ESP_ERR_TIMEOUT,.valid=1U<<TPS546_FIELD_TEMP,.read_temp1=110};
    expect_safety(unsafe,false,"temperature 110C");
    // A complete successful bus read can contain a latched overcurrent fault
    // with normal/zero telemetry because the TPS has already tripped itself.
    unsafe=(TPS546_StatusSnapshot){.valid=TPS546_SNAPSHOT_ALL_VALID,
        .status_word=TPS546_STATUS_IOUT_OC|TPS546_STATUS_OFF,
        .st_iout=TPS546_STATUS_IOUT_OCF,.operation=OPERATION_ON,
        .read_vin=5.15f,.read_vout=0,.read_iout=0,.read_temp1=70};
    expect_safety(unsafe,false,"status fault");
    assert(strstr(last_reason,"IOUT OC fault"));
    assert(g_telemetry_extrema.vin_valid && g_telemetry_extrema.vin_min_v==5.15f);
    assert(g_telemetry_extrema.tps_temp_valid && g_telemetry_extrema.tps_temp_max_c==70);
    assert(strstr(last_reason,"readback (not trip peak): VIN=5.150V VOUT=0.000V IOUT=0.000A"));
    assert(strstr(last_reason,"last validated TPS age=250ms VIN=5.1V VOUT=1.2V IOUT=29A VR=80C"));
    assert(!strstr(last_reason,"VIN=5.2V")); // Shutdown telemetry never replaces the event.
    unsafe.status_word=0; unsafe.st_iout=0; unsafe.read_vout=1.2f; unsafe.read_iout=34;
    expect_safety(unsafe,false,"IOUT 34.000A >= maximum 33.000A");
    assert(strstr(last_reason,"IOUT=34.000A"));
    assert(strstr(last_reason,"last validated TPS age=250ms VIN=5.1V VOUT=1.2V IOUT=29A"));
    // A partially read VIN is useful, but an unread temperature is not zero.
    unsafe.valid=1U<<TPS546_FIELD_VIN; unsafe.read_vin=4.7f;
    unsafe.read_error=ESP_ERR_TIMEOUT;
    expect_safety(unsafe,false,"VIN 4.700V <");
    assert(g_telemetry_extrema.vin_valid && g_telemetry_extrema.vin_min_v==4.7f);
    assert(!g_telemetry_extrema.tps_temp_valid);
    // Unread fields can contain garbage but never become measurements.
    reset(); state=(GlobalState){0}; failures_left=1; sample=unsafe; sample.valid=0;
    assert(capture_power_snapshot(&state,&out,false)==ESP_OK && !state.SYSTEM_MODULE.hardware_fault);
    reset(); sample.read_error=ESP_ERR_INVALID_ARG; failures_left=-1;
    assert(capture_power_snapshot(&state,&out,false)!=ESP_OK && read_calls==1 && delay_calls==0);
    // Runtime voltage sampling retries the read without repeating the write or restarting ASIC.
    reset(); state=(GlobalState){0}; failures_left=1;
    assert(bitaxe_gamma602_set_voltage_mv_for_config_unlocked(&state,1200,&config)==ESP_OK);
    assert(write_calls==1 && read_calls==2 && g_commanded_voltage_mv==1200 && g_regulator_enabled);
    reset(); write_result=ESP_ERR_TIMEOUT;
    assert(bitaxe_gamma602_set_voltage_mv_for_config_unlocked(&state,1200,&config)==ESP_ERR_TIMEOUT);
    assert(write_calls==1 && read_calls==0 && !g_regulator_enabled);
    reset(); write_result=ESP_ERR_INVALID_STATE;
    assert(bitaxe_gamma602_set_voltage_mv_for_config_unlocked(&state,1200,&config)==ESP_FAIL);
    assert(state.SYSTEM_MODULE.hardware_fault && read_calls==1 && write_calls==1);
    puts("TPS read-only grace, immediate safety faults, and single-write voltage tuning tests passed.");
}
