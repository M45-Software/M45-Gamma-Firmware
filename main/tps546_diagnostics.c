#include "tps546.h"

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

static const char *command_name(uint8_t command)
{
    switch (command) {
    case 0x79: return "STATUS_WORD";
    case 0x7a: return "STATUS_VOUT";
    case 0x7c: return "STATUS_INPUT";
    case 0x7b: return "STATUS_IOUT";
    case 0x7d: return "STATUS_TEMPERATURE";
    case 0x7e: return "STATUS_CML";
    case 0x80: return "STATUS_MFR_SPECIFIC";
    case 0x7f: return "STATUS_OTHER";
    case 0x01: return "OPERATION";
    case 0x02: return "ON_OFF_CONFIG";
    case 0x21: return "VOUT_COMMAND";
    case 0x8b: return "READ_VOUT";
    case 0x88: return "READ_VIN";
    case 0x8c: return "READ_IOUT";
    case 0x8d: return "READ_TEMPERATURE_1";
    default: return "unknown register";
    }
}

void TPS546_format_snapshot(const TPS546_StatusSnapshot *s, char *dst, size_t size)
{
    if (dst == NULL || size == 0) {
        return;
    }
    dst[0] = '\0';
    if (s == NULL) {
        append(dst, size, "TPS snapshot unavailable");
        return;
    }
    if (s->read_error != ESP_OK) {
        append(dst, size, "%s (0x%x) reading %s (0x%02x); partial snapshot; ",
               esp_err_to_name(s->read_error), (unsigned)s->read_error,
               command_name(s->failed_command), s->failed_command);
    }
#define VALID(field) (s->valid & (1U << TPS546_FIELD_##field))
#define FLAG(field, value, bit, text) \
    do { if (VALID(field) && ((value) & (bit))) append(dst, size, text "; "); } while (0)
    FLAG(STATUS, s->status_word, TPS546_STATUS_OFF, "output OFF");
    FLAG(STATUS, s->status_word, TPS546_STATUS_PGOOD, "power-good lost");
    FLAG(STATUS, s->status_word, TPS546_STATUS_VIN_UV, "VIN undervoltage");
    FLAG(STATUS, s->status_word, TPS546_STATUS_VOUT_OV, "VOUT overvoltage");
    FLAG(STATUS, s->status_word, TPS546_STATUS_IOUT_OC, "IOUT overcurrent");
    FLAG(STATUS, s->status_word, TPS546_STATUS_TEMP, "VR temperature flag");
    FLAG(STATUS, s->status_word, TPS546_STATUS_CML, "PMBus communication/logic flag");
    FLAG(VOUT_STATUS, s->st_vout, TPS546_STATUS_VOUT_OVF, "VOUT OV fault");
    FLAG(VOUT_STATUS, s->st_vout, TPS546_STATUS_VOUT_OVW, "VOUT OV warning");
    FLAG(VOUT_STATUS, s->st_vout, TPS546_STATUS_VOUT_UVF, "VOUT UV fault");
    FLAG(VOUT_STATUS, s->st_vout, TPS546_STATUS_VOUT_UVW, "VOUT UV warning");
    FLAG(VOUT_STATUS, s->st_vout, TPS546_STATUS_VOUT_MIN_MAX, "VOUT command out of range");
    FLAG(VOUT_STATUS, s->st_vout, TPS546_STATUS_VOUT_TON_MAX, "VOUT startup timeout");
    FLAG(INPUT_STATUS, s->st_input, TPS546_STATUS_VIN_OVF, "VIN OV fault");
    FLAG(INPUT_STATUS, s->st_input, TPS546_STATUS_VIN_UVW, "VIN UV warning");
    FLAG(INPUT_STATUS, s->st_input, TPS546_STATUS_VIN_LOW_VIN, "VIN low");
    FLAG(IOUT_STATUS, s->st_iout, TPS546_STATUS_IOUT_OCF, "IOUT OC fault");
    FLAG(IOUT_STATUS, s->st_iout, TPS546_STATUS_IOUT_OCW, "IOUT OC warning");
    FLAG(TEMP_STATUS, s->st_temp, TPS546_STATUS_TEMP_OTF, "VR overtemperature fault");
    FLAG(TEMP_STATUS, s->st_temp, TPS546_STATUS_TEMP_OTW, "VR overtemperature warning");
#undef FLAG
    // Keep the raw bytes even when the chip reports an unknown/reserved bit.
#define RAW(field, member, label, format) \
    do { if (VALID(field)) append(dst, size, label "=" format " ", s->member); } while (0)
    RAW(STATUS, status_word, "STATUS", "0x%04x");
    RAW(VOUT_STATUS, st_vout, "VOUT_STATUS", "0x%02x");
    RAW(INPUT_STATUS, st_input, "INPUT_STATUS", "0x%02x");
    RAW(IOUT_STATUS, st_iout, "IOUT_STATUS", "0x%02x");
    RAW(TEMP_STATUS, st_temp, "TEMP_STATUS", "0x%02x");
    RAW(CML_STATUS, st_cml, "CML", "0x%02x");
    RAW(MFR_STATUS, st_mfr, "MFR", "0x%02x");
    RAW(OTHER_STATUS, st_other, "OTHER", "0x%02x");
    RAW(OPERATION, operation, "OP", "0x%02x");
    RAW(CONFIG, on_off_config, "CFG", "0x%02x");
#undef RAW
    // PMBus telemetry is read after status registers. A hardware trip can
    // precede both reads, so these values cannot establish the trip's peak.
    append(dst, size, "readback (not trip peak):");
#define VALUE(field, member, label, format) \
    do { if (VALID(field)) append(dst, size, " " label "=" format, s->member); \
         else append(dst, size, " " label "=unavailable"); } while (0)
    VALUE(VIN, read_vin, "VIN", "%.3fV");
    VALUE(VOUT, read_vout, "VOUT", "%.3fV");
    VALUE(IOUT, read_iout, "IOUT", "%.3fA");
    VALUE(TEMP, read_temp1, "VR", "%dC");
    VALUE(COMMAND, vout_command, "command", "%.3fV");
#undef VALUE
#undef VALID
}
