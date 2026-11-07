#include "m45_fault_context.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static void append(char *dst, size_t size, const char *fmt, ...)
{
    const size_t used = strlen(dst);
    if (used >= size) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    vsnprintf(dst + used, size - used, fmt, args);
    va_end(args);
}

static void append_value(char *dst, size_t size, const char *label,
                          float value, bool valid, const char *unit)
{
    append(dst, size, " %s=", label);
    if (valid && isfinite(value)) {
        // Bounded significant digits keep even extreme diagnostic values from
        // consuming the context budget with hundreds of fixed-point digits.
        append(dst, size, "%.6g%s", (double)value, unit);
    } else {
        append(dst, size, "unavailable");
    }
}

static uint64_t sample_age_ms(uint64_t now_ms, uint64_t sample_ms)
{
    return now_ms >= sample_ms ? now_ms - sample_ms : 0;
}

void m45_fault_context_remember_power(m45_fault_context_t *context, uint64_t now_ms,
                                     float vin_v, float vout_v, float iout_a,
                                     float vr_temp_c)
{
    if (context == NULL || !isfinite(vin_v) || !isfinite(vout_v) ||
        !isfinite(iout_a) || !isfinite(vr_temp_c)) {
        return;
    }
    context->vin_v = vin_v;
    context->vout_v = vout_v;
    context->iout_a = iout_a;
    context->vr_temp_c = vr_temp_c;
    context->power_sample_ms = now_ms;
    context->power_valid = true;
}

void m45_fault_context_remember_asic_temp(m45_fault_context_t *context,
                                         uint64_t now_ms, float temp_c)
{
    if (context == NULL || !isfinite(temp_c)) {
        return;
    }
    context->asic_temp_c = temp_c;
    context->asic_temp_sample_ms = now_ms;
    context->asic_temp_valid = true;
}

void m45_fault_context_remember_settings(m45_fault_context_t *context, uint64_t now_ms,
                                        float actual_mhz, float requested_mhz,
                                        uint16_t requested_mv, uint16_t commanded_mv)
{
    if (context == NULL || !isfinite(actual_mhz) || actual_mhz <= 0 ||
        !isfinite(requested_mhz) || requested_mhz <= 0 ||
        requested_mv == 0 || commanded_mv == 0) {
        return;
    }
    context->actual_frequency_mhz = actual_mhz;
    context->requested_frequency_mhz = requested_mhz;
    context->requested_voltage_mv = requested_mv;
    context->commanded_voltage_mv = commanded_mv;
    context->operating_settings_valid = true;
    context->operating_settings_sample_ms = now_ms;
}

void m45_fault_context_format(const m45_fault_context_t *context, uint64_t now_ms,
                              const char *reason, char *dst, size_t size)
{
    if (dst == NULL || size == 0) {
        return;
    }
    static const m45_fault_context_t empty = {0};
    const m45_fault_context_t *c = context != NULL ? context : &empty;
    char details[M45_FAULT_CONTEXT_BYTES] = "operating context:";
    if (c->operating_settings_valid) {
        append(details, sizeof(details), " last operating age=%llums",
               (unsigned long long)sample_age_ms(now_ms, c->operating_settings_sample_ms));
    }
    append(details, sizeof(details), " clock");
    append_value(details, sizeof(details), "actual", c->actual_frequency_mhz,
                 c->actual_frequency_mhz > 0, "MHz");
    append_value(details, sizeof(details), "requested", c->requested_frequency_mhz,
                 c->requested_frequency_mhz > 0, "MHz");
    append(details, sizeof(details), "; core");
    append_value(details, sizeof(details), "requested", c->requested_voltage_mv,
                 c->requested_voltage_mv > 0, "mV");
    append_value(details, sizeof(details), "commanded", c->commanded_voltage_mv,
                 c->commanded_voltage_mv > 0, "mV");
    append(details, sizeof(details), "; last validated TPS");
    if (c->power_valid) {
        append(details, sizeof(details), " age=%llums",
               (unsigned long long)sample_age_ms(now_ms, c->power_sample_ms));
    }
    append_value(details, sizeof(details), "VIN", c->vin_v, c->power_valid, "V");
    append_value(details, sizeof(details), "VOUT", c->vout_v, c->power_valid, "V");
    append_value(details, sizeof(details), "IOUT", c->iout_a, c->power_valid, "A");
    append_value(details, sizeof(details), "VR", c->vr_temp_c, c->power_valid, "C");
    append(details, sizeof(details), "; last ASIC");
    if (c->asic_temp_valid) {
        append(details, sizeof(details), " age=%llums",
               (unsigned long long)sample_age_ms(now_ms, c->asic_temp_sample_ms));
    }
    append_value(details, sizeof(details), "temp", c->asic_temp_c,
                 c->asic_temp_valid, "C");

    const char *cause = reason != NULL ? reason : "Hardware fault";
    const size_t cause_len = strlen(cause);
    const size_t detail_len = strlen(details);
    const char truncated[] = " [reason truncated]";
    const size_t separator_len = 2; // "; "
    if (size <= detail_len + separator_len + sizeof(truncated)) {
        // Tiny diagnostic consumers still receive the original cause, safely.
        const size_t keep = cause_len < size - 1 ? cause_len : size - 1;
        memmove(dst, cause, keep);
        dst[keep] = '\0';
        return;
    }
    const size_t cause_capacity = size - 1 - detail_len - separator_len;
    const bool cut = cause_len > cause_capacity;
    const size_t keep = cut ? cause_capacity - (sizeof(truncated) - 1) : cause_len;
    memmove(dst, cause, keep);
    dst[keep] = '\0';
    if (cut) {
        append(dst, size, "%s", truncated);
    }
    append(dst, size, "; %s", details);
}
