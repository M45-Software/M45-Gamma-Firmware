#include "m45_telemetry_extrema.h"

#include <math.h>
#include <stddef.h>

void m45_telemetry_extrema_vin(m45_telemetry_extrema_t *e, float vin_v)
{
    if (e == NULL || !isfinite(vin_v) || vin_v < 0) {
        return;
    }
    if (!e->vin_valid || vin_v < e->vin_min_v) {
        e->vin_min_v = vin_v;
    }
    if (!e->vin_valid || vin_v > e->vin_max_v) {
        e->vin_max_v = vin_v;
    }
    e->vin_valid = true;
}

void m45_telemetry_extrema_tps_temp(m45_telemetry_extrema_t *e, float temp_c)
{
    if (e == NULL || !isfinite(temp_c)) {
        return;
    }
    if (!e->tps_temp_valid || temp_c > e->tps_temp_max_c) {
        e->tps_temp_max_c = temp_c;
    }
    e->tps_temp_valid = true;
}

void m45_telemetry_extrema_asic_temp(m45_telemetry_extrema_t *e, float temp_c)
{
    if (e == NULL || !isfinite(temp_c) || temp_c > 127.0f) {
        return;
    }
    if (!e->asic_temp_valid || temp_c > e->asic_temp_max_c) {
        e->asic_temp_max_c = temp_c;
    }
    e->asic_temp_valid = true;
}
