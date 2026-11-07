#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_STATE 3
#define ESP_LOGW(tag, ...) do { (void)(tag); if (0) printf(__VA_ARGS__); } while (0)
#define ESP_LOGE(tag, ...) ESP_LOGW(tag, __VA_ARGS__)
typedef struct {
    struct { int temp_offset; } DEVICE_CONFIG;
    struct { float chip_temp_avg; } POWER_MANAGEMENT_MODULE;
} GlobalState;
typedef struct { int safety_asic_temp_max_c; } m45_config_t;
static const char *TAG = "test";
static unsigned g_asic_temp_grace_until;
static int64_t now_us, g_asic_temp_unavailable_since_us = -1;
static float sensor;
static esp_err_t read_error;
static bool recovering, powered;
static unsigned shutdowns, remembered;
static char reason[256];
static const m45_config_t *m45_config_get(void) { static const m45_config_t c = {69}; return &c; }
static bool domain_reboot_recovery_pending(void) { return recovering; }
static unsigned xTaskGetTickCount(void) { return (unsigned)(now_us / 1000); }
static int64_t esp_timer_get_time(void) { return now_us; }
static esp_err_t bitaxe_fan_read_asic_temp_c(float *out) { *out = sensor; return read_error; }
static void log_asic_temp_no_reading(float temp) { (void)temp; }
static void remember_fault_asic_temperature(float temp) { (void)temp; ++remembered; }
static const char *esp_err_to_name(esp_err_t err) { (void)err; return "read failed"; }
static void safety_shutdownf(GlobalState *state, const char *fmt, ...) {
    (void)state; va_list args; va_start(args, fmt);
    vsnprintf(reason, sizeof(reason), fmt, args); va_end(args);
    powered = false; ++shutdowns;
}
/* TEMPERATURE_UNDER_TEST */
static void reset(void) {
    now_us = 0; g_asic_temp_grace_until = 0; g_asic_temp_unavailable_since_us = -1;
    sensor = 127.875f; read_error = ESP_OK; recovering = false; powered = true;
    shutdowns = remembered = 0; reason[0] = 0;
}
int main(void) {
    GlobalState state = {0}; float temp;
    reset();
    for (int i = 0; i < 20; ++i) {
        assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE);
        assert(powered && !shutdowns && !remembered);
        now_us += 250000;
    }
    assert(update_asic_temperature(&state, &temp) == ESP_FAIL);
    assert(!powered && shutdowns == 1 && strstr(reason, "unavailable for 5000ms"));
    reset(); update_asic_temperature(&state, &temp); now_us = 4000000;
    sensor = 60; assert(update_asic_temperature(&state, &temp) == ESP_OK);
    sensor = 127.875f; assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE);
    now_us = 8999000; assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE && powered);
    now_us = 9000000; assert(update_asic_temperature(&state, &temp) == ESP_FAIL && !powered);
    reset(); g_asic_temp_grace_until = 3000;
    assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE);
    now_us = 3000000; assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE);
    now_us = 7999000; assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE && powered);
    now_us = 8000000; assert(update_asic_temperature(&state, &temp) == ESP_FAIL);
    reset(); recovering = true; now_us = 60000000;
    assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE && powered);
    recovering = false; assert(update_asic_temperature(&state, &temp) == ESP_ERR_INVALID_STATE);
    now_us += 5000000; assert(update_asic_temperature(&state, &temp) == ESP_FAIL);
    reset(); sensor = 70;
    assert(update_asic_temperature(&state, &temp) == ESP_FAIL && !powered);
    assert(strstr(reason, "70.0C >= maximum 69.0C"));
    reset(); sensor = NAN;
    assert(update_asic_temperature(&state, &temp) == ESP_FAIL && !powered);
    reset(); read_error = ESP_FAIL;
    assert(update_asic_temperature(&state, &temp) == ESP_FAIL && !powered);
    puts("ASIC missing-temperature timeout, recovery reset, startup/recovery grace, and immediate hard faults passed.");
}
