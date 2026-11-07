#pragma once

#include "esp_err.h"
#include "m45_telemetry_extrema.h"

// Callers serialize saves/clears so a stale background save cannot undo a clear.
esp_err_t m45_telemetry_extrema_load(m45_telemetry_extrema_t *extrema);
esp_err_t m45_telemetry_extrema_save(const m45_telemetry_extrema_t *extrema);
