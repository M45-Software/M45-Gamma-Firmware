#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"
#include "m45_error_log.h"
#include "m45_fault_log.h"
#include "m45_fault_context.h"
#include "m45_telemetry_extrema.h"
#include "m45_log_buffer.h"
#include "tps546.h"

static int64_t boot_us = 32000000;
static time_t wall_time = 0;
static unsigned char stored[16384];
static size_t stored_size;
static unsigned char slot_data[8][2048];
static size_t slot_sizes[8];
static unsigned char ring_data[9][2048];
static size_t ring_sizes[9];
static unsigned write_calls, fail_write_at;
static bool fail_after_write;
static bool fail_reads;
static vprintf_like_t log_hook;
int64_t esp_timer_get_time(void) { return boot_us; }
time_t time(time_t *out) { if (out) *out = wall_time; return wall_time; }
const char *esp_err_to_name(esp_err_t err) { return err == ESP_ERR_TIMEOUT ? "ESP_ERR_TIMEOUT" : "ESP_FAIL"; }
esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *timer) { (void)args; *timer=(void *)1; return ESP_OK; }
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t us) { (void)t; (void)us; return ESP_OK; }
esp_err_t esp_timer_stop(esp_timer_handle_t t) { (void)t; return ESP_OK; }
static int sink(const char *fmt, va_list args) { return vsnprintf(NULL, 0, fmt, args); }
vprintf_like_t esp_log_set_vprintf(vprintf_like_t hook) { log_hook=hook; return sink; }
static void emit(const char *fmt, ...) { va_list args; va_start(args,fmt); log_hook(fmt,args); va_end(args); }
size_t strlcpy(char *dst,const char *src,size_t cap) { size_t n=strlen(src); if(cap){size_t len=n<cap-1?n:cap-1;memcpy(dst,src,len);dst[len]=0;}return n; }
esp_err_t nvs_open(const char *name,int mode,nvs_handle_t *h) { (void)name;(void)mode;*h=1;return ESP_OK; }
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *out,size_t *size) {
    if (fail_reads) return ESP_FAIL;
    (void)h; bool ring=strncmp(key,"ring",4)==0;
    int slot=ring?key[4]-'0':strncmp(key,"fault",5)==0?key[5]-'0':-1;
    size_t bytes=ring?ring_sizes[slot]:slot<0?stored_size:slot_sizes[slot];
    const void *data=ring?(void *)ring_data[slot]:slot<0?(void *)stored:(void *)slot_data[slot];
    if(!bytes)return ESP_ERR_NVS_NOT_FOUND;
    if(!out){*size=bytes;return ESP_OK;}
    if(*size<bytes)return ESP_ERR_NVS_INVALID_LENGTH;
    memcpy(out,data,bytes);*size=bytes;return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t size) {
    (void)h;bool ring=strncmp(key,"ring",4)==0;
    int slot=ring?key[4]-'0':strncmp(key,"fault",5)==0?key[5]-'0':-1;
    const bool failing=++write_calls==fail_write_at;
    if(failing && !fail_after_write)return ESP_FAIL;
    if(ring){assert(slot>=0 && slot<9 && size<=sizeof(ring_data[slot]));memcpy(ring_data[slot],data,size);ring_sizes[slot]=size;}
    else if(slot<0){assert(size<=sizeof(stored));memcpy(stored,data,size);stored_size=size;}
    else{assert(slot<8 && size<=sizeof(slot_data[slot]));memcpy(slot_data[slot],data,size);slot_sizes[slot]=size;}
    return failing?ESP_FAIL:ESP_OK;
}
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key) {
    (void)h;assert(strncmp(key,"fault",5)==0);int slot=key[5]-'0';assert(slot>=0 && slot<8);
    if(!slot_sizes[slot])return ESP_ERR_NVS_NOT_FOUND;
    slot_sizes[slot]=0;return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) { (void)h;return ESP_OK; }
void nvs_close(nvs_handle_t h) { (void)h; }

static void snapshot_tests(void)
{
    char text[1024];
    TPS546_StatusSnapshot s={.valid=TPS546_SNAPSHOT_ALL_VALID,
        .status_word=TPS546_STATUS_IOUT_OC|TPS546_STATUS_PGOOD,
        .st_iout=TPS546_STATUS_IOUT_OCF,.read_vin=4.91f,.read_vout=1.15f,
        .read_iout=33.4f,.read_temp1=88,.vout_command=1.16f};
    TPS546_format_snapshot(&s,text,sizeof(text));
    assert(strstr(text,"IOUT overcurrent"));assert(strstr(text,"power-good lost"));
    assert(strstr(text,"IOUT=33.400A"));assert(strstr(text,"VIN=4.910V"));assert(strstr(text,"IOUT_STATUS=0x80"));
    assert(strstr(text,"readback (not trip peak):"));
    s.valid=(1U<<TPS546_FIELD_STATUS);s.read_error=ESP_ERR_TIMEOUT;s.failed_command=0x7a;
    TPS546_format_snapshot(&s,text,sizeof(text));
    assert(strstr(text,"ESP_ERR_TIMEOUT (0x107) reading STATUS_VOUT (0x7a)"));
    assert(strstr(text,"VIN=unavailable"));assert(!strstr(text,"VIN=4.910V"));
    s.valid=TPS546_SNAPSHOT_ALL_VALID;s.read_error=ESP_OK;s.status_word=0xffff;
    s.st_vout=s.st_input=s.st_iout=s.st_temp=s.st_cml=s.st_mfr=s.st_other=0xff;
    TPS546_format_snapshot(&s,text,sizeof(text)-128);
    assert(strstr(text,"command=1.160V")); // Even simultaneous flags retain measurements.
    char tiny[4]={'x','x','x','x'};TPS546_format_snapshot(&s,tiny,sizeof(tiny));assert(tiny[3]==0);
    TPS546_format_snapshot(NULL,text,sizeof(text));assert(strstr(text,"unavailable"));
}

static void operating_context_tests(void)
{
    char text[M45_FAULT_MESSAGE_MAX + 1];
    m45_fault_context_t context = {
        .requested_frequency_mhz = 600,
        .requested_voltage_mv = 1175,
    };
    m45_fault_context_format(&context, 10000, "ASIC detection failed", text, sizeof(text));
    assert(strstr(text, "ASIC detection failed; operating context:"));
    assert(strstr(text, "actual=unavailable requested=600MHz"));
    assert(strstr(text, "requested=1175mV commanded=unavailable"));
    assert(strstr(text, "VIN=unavailable VOUT=unavailable IOUT=unavailable VR=unavailable"));
    assert(strstr(text, "last ASIC temp=unavailable"));
    assert(!strstr(text, "age=") && !strstr(text, "=0V"));

    m45_fault_context_remember_settings(&context, 10000, 599.5f, 600, 1175, 1180);
    m45_fault_context_remember_power(&context, 10000, 5.1f, 1.18f, 17.0f, 57.0f);
    m45_fault_context_remember_asic_temp(&context, 10250, 61.25f);
    m45_fault_context_format(&context, 11000, "TPS546 timeout", text, sizeof(text));
    assert(strstr(text, "actual=599.5MHz requested=600MHz"));
    assert(strstr(text, "last operating age=1000ms clock"));
    assert(strstr(text, "commanded=1180mV"));
    assert(strstr(text, "last validated TPS age=1000ms VIN=5.1V VOUT=1.18V IOUT=17A VR=57C"));
    assert(strstr(text, "last ASIC age=750ms temp=61.25C"));

    // An unread/non-finite sample must not replace the last measurement or its age.
    m45_fault_context_remember_power(&context, 12000, NAN, 0, 0, 0);
    m45_fault_context_remember_asic_temp(&context, 12000, INFINITY);
    // Reinitialization clears live frequency/voltage; retain the prior run.
    m45_fault_context_remember_settings(&context, 12000, 0, 525, 1150, 0);
    m45_fault_context_format(&context, 13000, "TPS546 timeout again", text, sizeof(text));
    assert(strstr(text, "last validated TPS age=3000ms VIN=5.1V"));
    assert(strstr(text, "last ASIC age=2750ms temp=61.25C"));
    assert(strstr(text, "last operating age=3000ms clock actual=599.5MHz requested=600MHz"));
    assert(strstr(text, "core requested=1175mV commanded=1180mV"));
    // A measured zero is distinct from an unavailable value.
    m45_fault_context_remember_power(&context, 13000, 5, 0, 0, 25);
    m45_fault_context_format(&context, 13000, "output off", text, sizeof(text));
    assert(strstr(text, "VOUT=0V IOUT=0A"));

    // Preserve context even with a full simultaneous-fault TPS diagnostic.
    TPS546_StatusSnapshot snapshot = {.valid=TPS546_SNAPSHOT_ALL_VALID,
        .status_word=0xffff, .st_vout=0xff, .st_input=0xff, .st_iout=0xff,
        .st_temp=0xff, .st_cml=0xff, .st_mfr=0xff, .st_other=0xff,
        .read_vin=5.1f, .read_vout=1.18f, .read_iout=33.4f,
        .read_temp1=110, .vout_command=1.18f};
    TPS546_format_snapshot(&snapshot, text, sizeof(text));
    m45_fault_context_format(&context, 13000, text, text, sizeof(text));
    assert(strstr(text, "output OFF; power-good lost"));
    assert(strstr(text, "command=1.180V"));
    assert(strstr(text, "last ASIC age=2750ms temp=61.25C"));
    assert(strlen(text) <= M45_FAULT_MESSAGE_MAX);

    // Capacity bound covers extreme values, maximum age, and in-place formatting.
    m45_fault_context_remember_settings(&context, 0, FLT_MAX, FLT_MAX, UINT16_MAX, UINT16_MAX);
    m45_fault_context_remember_power(&context, 0, -FLT_MAX, FLT_MAX, -FLT_MAX, FLT_MAX);
    m45_fault_context_remember_asic_temp(&context, 0, -FLT_MAX);
    memset(text, 'x', sizeof(text) - 1);
    text[sizeof(text) - 1] = 0;
    m45_fault_context_format(&context, UINT64_MAX, text, text, sizeof(text));
    assert(strstr(text, "[reason truncated]"));
    assert(strstr(text, "last operating age=18446744073709551615ms clock"));
    assert(strstr(text, "last ASIC age=18446744073709551615ms temp=-3.40282e+38C"));
    assert(strlen(text) == M45_FAULT_MESSAGE_MAX);
    char tiny[4] = "bad";
    m45_fault_context_format(&context, 0, "long reason", tiny, sizeof(tiny));
    assert(!strcmp(tiny, "lon"));
    m45_fault_context_format(NULL, 0, NULL, tiny, 1);
    assert(tiny[0] == 0);
    m45_fault_context_format(NULL, 0, NULL, NULL, 0);
}

static void extrema_tests(void)
{
    m45_telemetry_extrema_t e = {0};
    m45_telemetry_extrema_vin(&e, NAN);
    m45_telemetry_extrema_vin(&e, -1);
    m45_telemetry_extrema_tps_temp(&e, INFINITY);
    m45_telemetry_extrema_asic_temp(&e, 127.875f);
    assert(!e.vin_valid && !e.tps_temp_valid && !e.asic_temp_valid);
    m45_telemetry_extrema_vin(&e, 5.1f);
    m45_telemetry_extrema_vin(&e, 4.7f);
    m45_telemetry_extrema_vin(&e, 5.6f);
    m45_telemetry_extrema_vin(&e, 5.0f);
    assert(e.vin_min_v == 4.7f && e.vin_max_v == 5.6f);
    m45_telemetry_extrema_tps_temp(&e, 110);
    m45_telemetry_extrema_asic_temp(&e, 70);
    m45_telemetry_extrema_tps_temp(&e, 40);
    m45_telemetry_extrema_asic_temp(&e, 30);
    m45_telemetry_extrema_asic_temp(&e, 127.875f);
    assert(e.tps_temp_max_c == 110 && e.asic_temp_max_c == 70);
    m45_telemetry_extrema_asic_temp(&e, 127);
    assert(e.asic_temp_max_c == 127); // Same valid upper boundary as the safety path.
    m45_telemetry_extrema_vin(&e, 0); // A measured supply collapse is a real low.
    assert(e.vin_min_v == 0 && e.vin_max_v == 5.6f);
}

static void persistence_tests(void)
{
    typedef struct {uint32_t id;uint64_t epoch_seconds;uint32_t uptime_seconds;char message[96];} old_entry;
    struct {uint32_t magic,next_id;uint8_t count,reserved[3];old_entry entries[8];} old={.magic=0x4d343546,.next_id=8,.count=1};
    old.entries[0]=(old_entry){.id=7,.epoch_seconds=1786801200,.uptime_seconds=12,.message="old TPS fault"};
    memcpy(stored,&old,sizeof(old));stored_size=sizeof(old);
    assert(m45_fault_log_init()==ESP_OK);
    m45_fault_entry_t entries[8];assert(m45_fault_log_snapshot(entries,8)==1);
    assert(entries[0].id==7 && entries[0].epoch_seconds==1786801200 && entries[0].uptime_seconds==12);
    assert(!strcmp(entries[0].message,"old TPS fault"));
    char long_text[1024];memset(long_text,'x',sizeof(long_text));long_text[1023]=0;
    assert(m45_fault_log_record(long_text)==ESP_OK);
    assert(m45_fault_log_record(long_text)==ESP_OK); // Deduplicate the startup wrapper.
    assert(m45_fault_log_snapshot(entries,8)==2);assert(strlen(entries[0].message)==1023);
    assert(entries[0].epoch_seconds==0 && entries[0].uptime_seconds==32);
    wall_time=1786801500;boot_us+=6000000;
    assert(m45_fault_log_record("ASIC temperature 70.0C >= maximum 69.0C")==ESP_OK);
    assert(m45_fault_log_init()==ESP_OK); // Simulated restart: full text and dates survive.
    assert(m45_fault_log_snapshot(entries,8)==3);assert(entries[0].epoch_seconds==1786801500);
    assert(entries[0].uptime_seconds==38);assert(strlen(entries[1].message)==1023);
    // The same failure at the same uptime in a new boot is a separate event.
    assert(m45_fault_log_record("ASIC temperature 70.0C >= maximum 69.0C")==ESP_OK);
    assert(m45_fault_log_snapshot(entries,8)==4);
    assert(entries[0].id != entries[1].id);
    assert(m45_fault_log_remove(entries[1].id)==ESP_OK);
    assert(m45_fault_log_clear()==ESP_OK);assert(m45_fault_log_snapshot(entries,8)==0);
    for(int i=0;i<12;i++){boot_us+=6000000;long_text[0]=(char)('a'+i);assert(m45_fault_log_record(long_text)==ESP_OK);}
    assert(stored_size < 32); // Metadata never rewrites a full large history blob.
    size_t stored_total=stored_size;for(int i=0;i<8;i++)stored_total+=slot_sizes[i];
    for(int i=0;i<9;i++)stored_total+=ring_sizes[i];
    assert(stored_total<10000); // Eight records plus a staging slot fit in 24 KiB NVS.
    assert(m45_fault_log_init()==ESP_OK);assert(m45_fault_log_snapshot(entries,8)==8);
    assert(entries[0].message[0]=='l' && entries[7].message[0]=='e');
    assert(m45_fault_log_clear()==ESP_OK);assert(m45_fault_log_init()==ESP_OK);
    assert(m45_fault_log_snapshot(entries,8)==0);
}

typedef struct {
    unsigned char metadata[sizeof(stored)], legacy[sizeof(slot_data)], ring[sizeof(ring_data)];
    size_t metadata_size, legacy_sizes[8], sizes[9];
} test_nvs_image_t;

static void capture_nvs(test_nvs_image_t *image)
{
    memcpy(image->metadata, stored, sizeof(stored)); image->metadata_size = stored_size;
    memcpy(image->legacy, slot_data, sizeof(slot_data)); memcpy(image->legacy_sizes, slot_sizes, sizeof(slot_sizes));
    memcpy(image->ring, ring_data, sizeof(ring_data)); memcpy(image->sizes, ring_sizes, sizeof(ring_sizes));
}

static void restore_nvs(const test_nvs_image_t *image)
{
    memcpy(stored, image->metadata, sizeof(stored)); stored_size = image->metadata_size;
    memcpy(slot_data, image->legacy, sizeof(slot_data)); memcpy(slot_sizes, image->legacy_sizes, sizeof(slot_sizes));
    memcpy(ring_data, image->ring, sizeof(ring_data)); memcpy(ring_sizes, image->sizes, sizeof(ring_sizes));
    fail_write_at = write_calls = 0;
}

static void assert_ordered_history(size_t count)
{
    m45_fault_entry_t entries[8];
    assert(m45_fault_log_snapshot(entries, 8) == count);
    for (size_t i = 1; i < count; ++i) assert(entries[i-1].id > entries[i].id);
}

static void interrupted_history_tests(void)
{
    test_nvs_image_t *base = malloc(sizeof(*base)); assert(base);
    assert(m45_fault_log_clear() == ESP_OK);
    for (int i = 0; i < 8; ++i) {
        char message[32]; snprintf(message, sizeof(message), "baseline %d", i);
        assert(m45_fault_log_record(message) == ESP_OK);
    }
    capture_nvs(base);
    m45_fault_entry_t before[8], after[8];
    assert(m45_fault_log_snapshot(before, 8) == 8);
    // Fail before/after each of the two durable append writes. Restart sees
    // exactly the old history or its complete successor, never shifted copies.
    for (int applied = 0; applied < 2; ++applied) {
        for (unsigned at = 1; at <= 2; ++at) {
            restore_nvs(base); assert(m45_fault_log_init() == ESP_OK);
            fail_write_at = at; fail_after_write = applied;
            assert(m45_fault_log_record("newest fault") != ESP_OK);
            assert(m45_fault_log_snapshot(after, 8) == 8);
            assert(!memcmp(before, after, sizeof(before))); // Failed append preserves RAM.
            fail_write_at = 0;
            assert(m45_fault_log_init() == ESP_OK); assert_ordered_history(8);
            assert(m45_fault_log_snapshot(after, 8) == 8);
            if (after[0].id == before[0].id) {
                assert(!memcmp(before, after, sizeof(before)));
            } else {
                assert(after[0].id == before[0].id + 1);
                assert(!strcmp(after[0].message, "newest fault"));
                assert(!memcmp(before, after + 1, sizeof(before[0]) * 7));
            }
            assert(m45_fault_log_record("recovery fault") == ESP_OK);
            assert(m45_fault_log_init() == ESP_OK); assert_ordered_history(8);
        }
        restore_nvs(base); assert(m45_fault_log_init() == ESP_OK);
        fail_write_at = 1; fail_after_write = applied;
        assert(m45_fault_log_remove(before[3].id) != ESP_OK);
        fail_write_at = 0; assert(m45_fault_log_init() == ESP_OK);
        const size_t remaining = m45_fault_log_snapshot(after, 8);
        assert(remaining == (applied ? 7U : 8U)); assert_ordered_history(remaining);
        restore_nvs(base); assert(m45_fault_log_init() == ESP_OK);
        fail_write_at = 1; fail_after_write = applied;
        assert(m45_fault_log_clear() != ESP_OK);
        fail_write_at = 0; assert(m45_fault_log_init() == ESP_OK);
        assert_ordered_history(applied ? 0 : 8);
    }
    // Upgrade a full v0.0.14 slot store, interrupting each migration and append
    // write. Old keys stay valid until the ring's index has been published.
    struct { uint32_t magic, next_id; uint8_t count, reserved[3]; } metadata = {
        .magic = 0x4d343547, .next_id = 9, .count = 8,
    };
    memcpy(stored, &metadata, sizeof(metadata)); stored_size = sizeof(metadata);
    memset(ring_sizes, 0, sizeof(ring_sizes));
    for (int i = 0; i < 8; ++i) {
        m45_fault_entry_t entry = {.id = (uint32_t)i + 1, .uptime_seconds = 100};
        snprintf(entry.message, sizeof(entry.message), "old slot %d", i);
        slot_sizes[i] = offsetof(m45_fault_entry_t, message) + strlen(entry.message) + 1;
        memcpy(slot_data[i], &entry, slot_sizes[i]);
    }
    capture_nvs(base);
    for (int applied = 0; applied < 2; ++applied) {
        for (unsigned at = 1; at <= 3; ++at) {
            restore_nvs(base); assert(m45_fault_log_init() == ESP_OK);
            fail_write_at = at; fail_after_write = applied;
            assert(m45_fault_log_record("upgraded fault") != ESP_OK);
            fail_write_at = 0;
            assert(m45_fault_log_init() == ESP_OK); assert_ordered_history(8);
            assert(m45_fault_log_snapshot(after, 8) == 8);
            const uint32_t newest = after[0].id;
            assert(newest == 8 || newest == 9);
            for (size_t i = 0; i < 8; ++i) assert(after[i].id == newest - i);
        }
    }
    restore_nvs(base);
    // Recover the duplicate-ID shape produced by the old interrupted rotation.
    memcpy(slot_data[0], slot_data[1], slot_sizes[1]); slot_sizes[0] = slot_sizes[1];
    assert(m45_fault_log_init() == ESP_OK); assert_ordered_history(7);
    assert(m45_fault_log_record("after duplicate recovery") == ESP_OK);
    assert(write_calls == 3); // Index migration plus two append writes; no full copy.
    assert(m45_fault_log_init() == ESP_OK); assert_ordered_history(8);
    fail_reads = true;
    assert(m45_fault_log_init() == ESP_FAIL);
    const unsigned writes_before = write_calls;
    assert(m45_fault_log_record("storage unavailable") == ESP_FAIL);
    assert(write_calls == writes_before); // A failed restore never publishes empty history.
    fail_reads = false;
    assert_ordered_history(8); // Snapshot retries the failed load without losing records.
    fail_reads = true;
    assert(m45_fault_log_init() == ESP_FAIL);
    assert(m45_fault_log_clear() == ESP_OK); // Explicit clear can recover unreadable history.
    fail_reads = false;
    assert(m45_fault_log_init() == ESP_OK); assert_ordered_history(0);
    for (int i = 0; i < 8; ++i) assert(slot_sizes[i] == 0);
    free(base);
}

static void error_capture_tests(void)
{
    boot_us = 38000000;
    m45_log_buffer_init();
    m45_error_entry_t *entries=calloc(M45_ERROR_LOG_CAPACITY,sizeof(*entries));assert(entries);
    uint32_t dropped,truncated;
    emit("I (1) test: normal log\n");emit("W (2) test: warning\n");
    emit("\033[0;31mE (3) tps546: timeout %d\033[0m\n",7);
    assert(m45_error_log_snapshot(entries,64,&dropped,&truncated)==1);
    assert(strstr(entries[0].message,"timeout 7"));assert(!strchr(entries[0].message,'\033'));
    assert(entries[0].epoch_seconds==(uint64_t)wall_time && entries[0].uptime_seconds==38);
    assert(!dropped && !truncated);
    char live[128];assert(m45_log_buffer_copy_since(0,live,sizeof(live),NULL,NULL)==0);
    m45_log_buffer_keep_active(5000);emit("E (4) test: page open\n");
    assert(m45_log_buffer_copy_since(0,live,sizeof(live),NULL,NULL)>0);
    boot_us+=6000000;wall_time=0;emit("E (5) test: page closed again\n");
    assert(m45_error_log_snapshot(entries,64,&dropped,&truncated)==3);
    assert(entries[0].epoch_seconds==0 && entries[0].uptime_seconds==44);
    char detailed[1200];memset(detailed,'b',sizeof(detailed)-1);detailed[1199]=0;
    emit("E (6) test: %s end-of-details",detailed);
    assert(m45_error_log_snapshot(entries,64,&dropped,&truncated)==4);
    assert(strstr(entries[0].message,"end-of-details"));assert(truncated==0);
    char oversized[3000];memset(oversized,'a',sizeof(oversized)-1);oversized[2999]=0;
    emit("E (6) test: %s",oversized);
    assert(m45_error_log_snapshot(entries,64,&dropped,&truncated)==5);assert(truncated==1);
    assert(strstr(entries[0].message,"[message truncated]"));
    for(int i=0;i<70;i++)emit("E (%d) test: entry %d\n",i,i);
    assert(m45_error_log_snapshot(entries,64,&dropped,&truncated)==64);assert(dropped==11);
    assert(strstr(entries[0].message,"entry 69"));assert(strstr(entries[63].message,"entry 6"));
    free(entries);
}
int main(void){snapshot_tests();operating_context_tests();extrema_tests();persistence_tests();interrupted_history_tests();error_capture_tests();puts("Fault diagnostics, extrema, atomic history/migration interruption, timestamps, and always-on error capture passed.");}
