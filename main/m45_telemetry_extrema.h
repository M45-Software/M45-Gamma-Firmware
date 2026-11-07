#pragma once

#include <stdbool.h>

// Raw observations since the last manual clear, independent of live averages.
// Validity distinguishes an unobserved value from a measured zero.
typedef struct {
    bool vin_valid;
    bool tps_temp_valid;
    bool asic_temp_valid;
    float vin_min_v;
    float vin_max_v;
    float tps_temp_max_c;
    float asic_temp_max_c;
} m45_telemetry_extrema_t;

void m45_telemetry_extrema_vin(m45_telemetry_extrema_t *extrema, float vin_v);
void m45_telemetry_extrema_tps_temp(m45_telemetry_extrema_t *extrema, float temp_c);
void m45_telemetry_extrema_asic_temp(m45_telemetry_extrema_t *extrema, float temp_c);
