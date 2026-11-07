#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "m45_telemetry_extrema_store.h"
#include "nvs.h"

static uint8_t committed[64], pending[64];
static size_t committed_bytes, pending_bytes;
static unsigned commits, writes;
static bool fail_commit, fail_read, g_extrema_restore_pending;
static int critical_depth, g_power_snapshot_lock;
static pthread_mutex_t g_extrema_persist_lock = PTHREAD_MUTEX_INITIALIZER;
static m45_telemetry_extrema_t g_telemetry_extrema;
static const char *TAG = "test";
#define portENTER_CRITICAL(lock) do { (void)(lock); ++critical_depth; } while (0)
#define portEXIT_CRITICAL(lock) do { (void)(lock); --critical_depth; } while (0)
#define ESP_LOGW(tag, ...) do { (void)(tag); if (0) printf(__VA_ARGS__); } while (0)
const char *esp_err_to_name(esp_err_t err) { (void)err; return "storage failure"; }

esp_err_t nvs_open(const char *name, int mode, nvs_handle_t *out) {
    assert(!critical_depth && !strcmp(name, "m45_extrema"));
    if (mode == NVS_READONLY && !committed_bytes) return ESP_ERR_NVS_NOT_FOUND;
    *out = 1; return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t nvs, const char *key, void *dst, size_t *size) {
    (void)nvs; assert(!strcmp(key, "values"));
    if (fail_read) return ESP_FAIL;
    if (!committed_bytes) return ESP_ERR_NVS_NOT_FOUND;
    if (*size < committed_bytes) return ESP_ERR_NVS_INVALID_LENGTH;
    memcpy(dst, committed, committed_bytes); *size = committed_bytes; return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t nvs, const char *key, const void *src, size_t size) {
    (void)nvs; assert(!critical_depth && !strcmp(key, "values") && size <= sizeof(pending));
    memcpy(pending, src, size); pending_bytes = size; ++writes; return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t nvs) {
    (void)nvs; assert(!critical_depth); ++commits;
    if (fail_commit) return ESP_FAIL;
    memcpy(committed, pending, pending_bytes); committed_bytes = pending_bytes; return ESP_OK;
}
void nvs_close(nvs_handle_t nvs) { (void)nvs; }

/* EXTREMA_CONTROL_UNDER_TEST */

int main(void) {
    m45_telemetry_extrema_t loaded;
    assert(m45_telemetry_extrema_load(&loaded) == ESP_OK);
    assert(!loaded.vin_valid && !loaded.tps_temp_valid && !loaded.asic_temp_valid);
    g_telemetry_extrema = loaded;
    m45_telemetry_extrema_vin(&g_telemetry_extrema, 4.7f);
    m45_telemetry_extrema_vin(&g_telemetry_extrema, 5.6f);
    m45_telemetry_extrema_tps_temp(&g_telemetry_extrema, 110);
    m45_telemetry_extrema_asic_temp(&g_telemetry_extrema, 70);
    persist_telemetry_extrema();
    assert(commits == 1 && writes == 1 && committed_bytes == 24);
    persist_telemetry_extrema(); // Unchanged periodic saves never write flash.
    assert(commits == 1 && writes == 1);
    assert(m45_telemetry_extrema_load(&loaded) == ESP_OK); // Simulated reboot.
    assert(loaded.vin_valid && loaded.vin_min_v == 4.7f && loaded.vin_max_v == 5.6f);
    assert(loaded.tps_temp_valid && loaded.tps_temp_max_c == 110);
    assert(loaded.asic_temp_valid && loaded.asic_temp_max_c == 70);

    fail_commit = true;
    assert(bitaxe_gamma602_clear_telemetry_extrema() == ESP_FAIL);
    assert(g_telemetry_extrema.vin_min_v == 4.7f && g_telemetry_extrema.asic_temp_max_c == 70);
    assert(m45_telemetry_extrema_load(&loaded) == ESP_OK && loaded.vin_valid);
    fail_commit = false;
    assert(bitaxe_gamma602_clear_telemetry_extrema() == ESP_OK);
    assert(!g_telemetry_extrema.vin_valid && !g_telemetry_extrema.asic_temp_valid);
    persist_telemetry_extrema(); // Background flush cannot restore old extremes.
    assert(m45_telemetry_extrema_load(&loaded) == ESP_OK);
    assert(!loaded.vin_valid && !loaded.tps_temp_valid && !loaded.asic_temp_valid);

    m45_telemetry_extrema_vin(&g_telemetry_extrema, 5.0f);
    m45_telemetry_extrema_asic_temp(&g_telemetry_extrema, 40);
    fail_commit = true;
    persist_telemetry_extrema();
    assert(g_telemetry_extrema.asic_temp_max_c == 40);
    fail_commit = false;
    persist_telemetry_extrema(); // Failed writes retry; new period persists.
    assert(m45_telemetry_extrema_load(&loaded) == ESP_OK);
    assert(loaded.vin_min_v == 5.0f && loaded.vin_max_v == 5.0f);
    assert(loaded.asic_temp_max_c == 40 && !loaded.tps_temp_valid);

    // A failed boot read must block writes until the old record can be merged.
    m45_telemetry_extrema_tps_temp(&g_telemetry_extrema, 110);
    persist_telemetry_extrema();
    fail_read = true;
    assert(m45_telemetry_extrema_load(&g_telemetry_extrema) == ESP_FAIL);
    g_extrema_restore_pending = true;
    m45_telemetry_extrema_tps_temp(&g_telemetry_extrema, 80);
    m45_telemetry_extrema_vin(&g_telemetry_extrema, 4.6f);
    const unsigned writes_before = writes;
    persist_telemetry_extrema();
    assert(writes == writes_before && g_extrema_restore_pending);
    fail_read = false;
    persist_telemetry_extrema();
    assert(!g_extrema_restore_pending && g_telemetry_extrema.tps_temp_max_c == 110);
    assert(m45_telemetry_extrema_load(&loaded) == ESP_OK);
    assert(loaded.tps_temp_max_c == 110 && loaded.vin_min_v == 4.6f && loaded.vin_max_v == 5.0f);
    // An explicit manual clear is allowed even while a restore is pending.
    g_extrema_restore_pending = true;
    assert(bitaxe_gamma602_clear_telemetry_extrema() == ESP_OK);
    assert(!g_extrema_restore_pending);

    committed[0] ^= 1; // Reject incompatible/corrupted records safely.
    assert(m45_telemetry_extrema_load(&loaded) != ESP_OK);
    assert(!loaded.vin_valid && !loaded.tps_temp_valid && !loaded.asic_temp_valid);
    puts("Persistent extrema survive reboot; clears, failed commits, retries, and unchanged saves passed.");
}
