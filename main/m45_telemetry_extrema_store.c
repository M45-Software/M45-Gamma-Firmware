#include "m45_telemetry_extrema_store.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "nvs.h"

#define EXTREMA_MAGIC 0x4d343501U
#define EXTREMA_NAMESPACE "m45_extrema"
#define EXTREMA_KEY "values"

typedef struct {
    uint32_t magic;
    uint8_t flags;
    uint8_t reserved[3];
    float vin_min_v, vin_max_v, tps_temp_max_c, asic_temp_max_c;
} extrema_store_t;

_Static_assert(sizeof(extrema_store_t) == 24, "extrema storage layout");
static extrema_store_t g_saved;
static bool g_saved_valid;

static extrema_store_t encode(const m45_telemetry_extrema_t *e)
{
    extrema_store_t s = {.magic = EXTREMA_MAGIC};
    if (e->vin_valid) {
        s.flags |= 1;
        s.vin_min_v = e->vin_min_v;
        s.vin_max_v = e->vin_max_v;
    }
    if (e->tps_temp_valid) {
        s.flags |= 2;
        s.tps_temp_max_c = e->tps_temp_max_c;
    }
    if (e->asic_temp_valid) {
        s.flags |= 4;
        s.asic_temp_max_c = e->asic_temp_max_c;
    }
    return s;
}

esp_err_t m45_telemetry_extrema_load(m45_telemetry_extrema_t *e)
{
    memset(e, 0, sizeof(*e));
    g_saved_valid = false;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(EXTREMA_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        g_saved = encode(e);
        g_saved_valid = true;
        return ESP_OK;
    }
    if (err != ESP_OK) return err;
    extrema_store_t s = {0};
    size_t bytes = sizeof(s);
    err = nvs_get_blob(nvs, EXTREMA_KEY, &s, &bytes);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        g_saved = encode(e);
        g_saved_valid = true;
        return ESP_OK;
    }
    if (err != ESP_OK) return err;
    if (bytes != sizeof(s) || s.magic != EXTREMA_MAGIC || (s.flags & ~7U) ||
        ((s.flags & 1) && (!isfinite(s.vin_min_v) || !isfinite(s.vin_max_v) ||
                          s.vin_min_v < 0 || s.vin_min_v > s.vin_max_v)) ||
        ((s.flags & 2) && !isfinite(s.tps_temp_max_c)) ||
        ((s.flags & 4) && (!isfinite(s.asic_temp_max_c) || s.asic_temp_max_c > 127))) {
        return ESP_ERR_INVALID_SIZE;
    }
    *e = (m45_telemetry_extrema_t){
        .vin_valid = (s.flags & 1) != 0,
        .tps_temp_valid = (s.flags & 2) != 0,
        .asic_temp_valid = (s.flags & 4) != 0,
        .vin_min_v = s.vin_min_v, .vin_max_v = s.vin_max_v,
        .tps_temp_max_c = s.tps_temp_max_c, .asic_temp_max_c = s.asic_temp_max_c,
    };
    g_saved = encode(e);
    g_saved_valid = true;
    return ESP_OK;
}

esp_err_t m45_telemetry_extrema_save(const m45_telemetry_extrema_t *e)
{
    const extrema_store_t s = encode(e);
    if (g_saved_valid && memcmp(&g_saved, &s, sizeof(s)) == 0) return ESP_OK;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(EXTREMA_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(nvs, EXTREMA_KEY, &s, sizeof(s));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err == ESP_OK) {
        g_saved = s;
        g_saved_valid = true;
    }
    return err;
}
