#include "m45_fault_log.h"

#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_timer.h"
#include "nvs.h"

#define FAULT_NAMESPACE "m45_faults"
#define FAULT_KEY "history"
#define FAULT_MAGIC 0x4d343546U
#define VALID_WALL_TIME_EPOCH 1704067200ULL

typedef struct {
    uint32_t magic;
    uint32_t next_id;
    uint8_t count;
    uint8_t reserved[3];
    m45_fault_entry_t entries[M45_FAULT_LOG_CAPACITY];
} fault_store_t;

typedef struct {
    uint32_t magic;
    uint32_t next_id;
    uint8_t count;
    uint8_t reserved[3];
} fault_metadata_t;

#define FAULT_METADATA_MAGIC (FAULT_MAGIC + 1U)
#define FAULT_RING_MAGIC (FAULT_MAGIC + 2U)
#define FAULT_RING_SLOTS (M45_FAULT_LOG_CAPACITY + 1U)
#define FAULT_LEGACY_SLOT_BASE 16U
typedef struct {
    uint32_t magic;
    uint32_t next_id;
    uint8_t count;
    uint8_t slots[M45_FAULT_LOG_CAPACITY];
    uint8_t reserved[3];
} fault_ring_metadata_t;
_Static_assert(M45_FAULT_LOG_CAPACITY <= 8, "fault slot key buffer capacity");
_Static_assert(FAULT_RING_SLOTS <= 10, "single-digit fault ring keys");

static pthread_mutex_t g_fault_lock = PTHREAD_MUTEX_INITIALIZER;
static fault_store_t g_faults;
static uint32_t g_last_recorded_id_this_boot;
static fault_ring_metadata_t g_ring;
static bool g_ring_valid;
static bool g_loaded;
static bool g_legacy_slots;
static uint8_t g_legacy_slot_indices[M45_FAULT_LOG_CAPACITY];
static esp_err_t fault_store_load_locked(void);

static void fault_store_reset(void)
{
    memset(&g_faults, 0, sizeof(g_faults));
    g_faults.magic = FAULT_MAGIC;
    g_faults.next_id = 1;
}

static esp_err_t write_ring_entry(nvs_handle_t nvs, uint8_t slot,
                                  const m45_fault_entry_t *entry)
{
    char key[] = "ring0";
    key[4] += slot;
    const size_t bytes = offsetof(m45_fault_entry_t, message) + strlen(entry->message) + 1;
    return nvs_set_blob(nvs, key, entry, bytes);
}

static esp_err_t publish_ring(nvs_handle_t nvs, const fault_ring_metadata_t *next)
{
    // NVS updates individual keys atomically; commit is not a transaction over
    // several keys. Only this small index publishes staged records/removals.
    esp_err_t err = nvs_set_blob(nvs, FAULT_KEY, next, sizeof(*next));
    if (err == ESP_OK) err = nvs_commit(nvs);
    if (err == ESP_OK) {
        g_ring = *next;
        g_ring_valid = true;
    }
    return err;
}

static void cleanup_legacy_slots(nvs_handle_t nvs)
{
    for (size_t i = 0; i < M45_FAULT_LOG_CAPACITY; ++i) {
        bool referenced = false;
        for (size_t j = 0; j < g_ring.count; ++j) {
            if (g_ring.slots[j] == FAULT_LEGACY_SLOT_BASE + i) referenced = true;
        }
        if (referenced) continue;
        char key[] = "fault0";
        key[5] += i;
        (void)nvs_erase_key(nvs, key);
    }
}

static esp_err_t ensure_ring_locked(nvs_handle_t nvs)
{
    if (!g_loaded) return ESP_FAIL;
    if (g_ring_valid) return ESP_OK;
    fault_ring_metadata_t next = {
        .magic = FAULT_RING_MAGIC, .next_id = g_faults.next_id, .count = g_faults.count,
    };
    // Reuse old slot records in place: copying a full 8 KiB history before
    // publication could exhaust the small NVS partition during an upgrade.
    for (size_t i = 0; i < g_faults.count; ++i) {
        if (g_legacy_slots) {
            next.slots[i] = FAULT_LEGACY_SLOT_BASE + g_legacy_slot_indices[i];
            continue;
        }
        next.slots[i] = (uint8_t)i;
        const esp_err_t err = write_ring_entry(nvs, next.slots[i], &g_faults.entries[i]);
        if (err != ESP_OK) return err;
    }
    const esp_err_t err = publish_ring(nvs, &next);
    if (err == ESP_OK) {
        // Old keys are no longer authoritative. Cleanup is best-effort and
        // interrupted cleanup cannot affect the new index or its records.
        cleanup_legacy_slots(nvs);
    }
    return err;
}

static esp_err_t open_ring_locked(nvs_handle_t *nvs)
{
    if (!g_loaded) {
        const esp_err_t load_err = fault_store_load_locked();
        if (load_err != ESP_OK) return load_err;
    }
    esp_err_t err = nvs_open(FAULT_NAMESPACE, NVS_READWRITE, nvs);
    if (err != ESP_OK) return err;
    err = ensure_ring_locked(*nvs);
    if (err == ESP_OK) cleanup_legacy_slots(*nvs);
    if (err != ESP_OK) nvs_close(*nvs);
    return err;
}

static esp_err_t fault_store_load_locked(void)
{
    fault_store_reset();
    g_ring_valid = false;
    g_loaded = false;
    g_legacy_slots = false;
    g_last_recorded_id_this_boot = 0;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(FAULT_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        g_loaded = true;
        return ESP_OK;
    }
    if (err == ESP_OK) {
        size_t size = 0;
        err = nvs_get_blob(nvs, FAULT_KEY, NULL, &size);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            nvs_close(nvs);
            g_loaded = true;
            return ESP_OK;
        }
        if (err == ESP_OK && size == sizeof(fault_ring_metadata_t)) {
            fault_ring_metadata_t metadata;
            err = nvs_get_blob(nvs, FAULT_KEY, &metadata, &size);
            if (err == ESP_OK && (metadata.magic != FAULT_RING_MAGIC ||
                metadata.count > M45_FAULT_LOG_CAPACITY || metadata.next_id == 0)) {
                err = ESP_ERR_INVALID_SIZE;
            }
            uint32_t used = 0;
            for (size_t i = 0; err == ESP_OK && i < metadata.count; ++i) {
                const uint8_t slot = metadata.slots[i];
                const bool legacy = slot >= FAULT_LEGACY_SLOT_BASE &&
                                    slot < FAULT_LEGACY_SLOT_BASE + M45_FAULT_LOG_CAPACITY;
                if ((!legacy && slot >= FAULT_RING_SLOTS) || (used & (1U << slot))) {
                    err = ESP_ERR_INVALID_SIZE;
                    break;
                }
                used |= 1U << slot;
                char key[7];
                if (legacy) {
                    memcpy(key, "fault0", 7);
                    key[5] += slot - FAULT_LEGACY_SLOT_BASE;
                } else {
                    memcpy(key, "ring0", 6);
                    key[4] += slot;
                }
                size_t bytes = sizeof(g_faults.entries[i]);
                err = nvs_get_blob(nvs, key, &g_faults.entries[i], &bytes);
                if (err == ESP_OK && (bytes <= offsetof(m45_fault_entry_t, message) ||
                    bytes > offsetof(m45_fault_entry_t, message) + M45_FAULT_MESSAGE_MAX + 1 ||
                    ((char *)&g_faults.entries[i])[bytes - 1] != '\0' ||
                    g_faults.entries[i].id == 0)) err = ESP_ERR_INVALID_SIZE;
                for (size_t j = 0; err == ESP_OK && j < i; ++j) {
                    if (g_faults.entries[j].id == g_faults.entries[i].id) err = ESP_ERR_INVALID_SIZE;
                }
            }
            if (err == ESP_OK) {
                g_faults.count = metadata.count;
                g_faults.next_id = metadata.next_id;
                g_ring = metadata;
                g_ring_valid = true;
                size = sizeof(g_faults);
            }
        } else if (err == ESP_OK && size == sizeof(fault_metadata_t)) {
            fault_metadata_t metadata;
            err = nvs_get_blob(nvs, FAULT_KEY, &metadata, &size);
            if (err == ESP_OK && metadata.magic == FAULT_METADATA_MAGIC &&
                metadata.count <= M45_FAULT_LOG_CAPACITY && metadata.next_id != 0) {
                g_faults.count = metadata.count;
                g_faults.next_id = metadata.next_id;
                g_legacy_slots = true;
                for (size_t i = 0; err == ESP_OK && i < metadata.count; ++i) {
                    g_legacy_slot_indices[i] = (uint8_t)i;
                    char key[] = "fault0";
                    key[5] += (char)i;
                    size_t bytes = sizeof(g_faults.entries[i]);
                    err = nvs_get_blob(nvs, key, &g_faults.entries[i], &bytes);
                    if (err == ESP_ERR_NVS_NOT_FOUND) {
                        err = ESP_ERR_INVALID_SIZE;
                    }
                    if (err == ESP_OK &&
                        (bytes <= offsetof(m45_fault_entry_t, message) ||
                         bytes > offsetof(m45_fault_entry_t, message) + M45_FAULT_MESSAGE_MAX + 1 ||
                         ((char *)&g_faults.entries[i])[bytes - 1] != '\0')) {
                        err = ESP_ERR_INVALID_SIZE;
                    }
                }
                size = sizeof(g_faults);
            }
        } else if (err == ESP_OK && size == sizeof(g_faults)) {
            err = nvs_get_blob(nvs, FAULT_KEY, &g_faults, &size);
        } else if (err == ESP_OK) {
            // Import v0.0.11/v0.0.12 history without losing dates or IDs.
            typedef struct {
                uint32_t id;
                uint64_t epoch_seconds;
                uint32_t uptime_seconds;
                char message[96];
            } legacy_entry_t;
            typedef struct {
                uint32_t magic;
                uint32_t next_id;
                uint8_t count;
                uint8_t reserved[3];
                legacy_entry_t entries[M45_FAULT_LOG_CAPACITY];
            } legacy_store_t;
            legacy_store_t *legacy = size == sizeof(legacy_store_t)
                                         ? malloc(sizeof(*legacy)) : NULL;
            if (legacy != NULL) {
                err = nvs_get_blob(nvs, FAULT_KEY, legacy, &size);
                if (err == ESP_OK && legacy->magic == FAULT_MAGIC &&
                    legacy->count <= M45_FAULT_LOG_CAPACITY && legacy->next_id != 0) {
                    g_faults.count = legacy->count;
                    g_faults.next_id = legacy->next_id;
                    for (size_t i = 0; i < legacy->count; ++i) {
                        g_faults.entries[i].id = legacy->entries[i].id;
                        g_faults.entries[i].epoch_seconds = legacy->entries[i].epoch_seconds;
                        g_faults.entries[i].uptime_seconds = legacy->entries[i].uptime_seconds;
                        legacy->entries[i].message[95] = '\0';
                        strlcpy(g_faults.entries[i].message, legacy->entries[i].message,
                                sizeof(g_faults.entries[i].message));
                    }
                    size = sizeof(g_faults);
                }
                free(legacy);
            }
        }
        nvs_close(nvs);
        if (err != ESP_OK || size != sizeof(g_faults) || g_faults.magic != FAULT_MAGIC ||
            g_faults.count > M45_FAULT_LOG_CAPACITY || g_faults.next_id == 0) {
            fault_store_reset();
            if (err == ESP_OK) err = ESP_ERR_INVALID_SIZE;
        }
    }
    for (size_t i = 0; i < g_faults.count; ++i) {
        g_faults.entries[i].message[M45_FAULT_MESSAGE_MAX] = '\0';
    }
    // Old interrupted rotations could leave the same ID in adjacent slots.
    // Keep its latest occurrence and preserve the original slot mapping during
    // migration, rather than publishing duplicate IDs in the new index.
    if (err == ESP_OK && !g_ring_valid) {
        for (size_t i = 0; i < g_faults.count;) {
            bool duplicate = false;
            for (size_t j = i + 1; j < g_faults.count; ++j) {
                if (g_faults.entries[i].id == g_faults.entries[j].id) duplicate = true;
            }
            if (duplicate) {
                const size_t following = g_faults.count - i - 1;
                memmove(g_faults.entries + i, g_faults.entries + i + 1,
                        following * sizeof(g_faults.entries[0]));
                memmove(g_legacy_slot_indices + i, g_legacy_slot_indices + i + 1, following);
                --g_faults.count;
            } else {
                ++i;
            }
        }
    }
    g_loaded = err == ESP_OK;
    return err;
}

esp_err_t m45_fault_log_init(void)
{
    pthread_mutex_lock(&g_fault_lock);
    const esp_err_t err = fault_store_load_locked();
    pthread_mutex_unlock(&g_fault_lock);
    return err;
}

esp_err_t m45_fault_log_record(const char *message)
{
    if (message == NULL || message[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    pthread_mutex_lock(&g_fault_lock);
    const uint32_t uptime_seconds = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (g_faults.count > 0) {
        const m45_fault_entry_t *latest = &g_faults.entries[g_faults.count - 1U];
        if (latest->id == g_last_recorded_id_this_boot &&
            strcmp(latest->message, message) == 0 &&
            uptime_seconds >= latest->uptime_seconds &&
            uptime_seconds - latest->uptime_seconds <= 5U) {
            pthread_mutex_unlock(&g_fault_lock);
            return ESP_OK;
        }
    }
    nvs_handle_t nvs;
    esp_err_t err = open_ring_locked(&nvs);
    if (err != ESP_OK) {
        pthread_mutex_unlock(&g_fault_lock);
        return err;
    }
    m45_fault_entry_t entry = {0};
    entry.id = g_faults.next_id;
    const time_t now = time(NULL);
    entry.epoch_seconds = now >= (time_t)VALID_WALL_TIME_EPOCH ? (uint64_t)now : 0;
    entry.uptime_seconds = uptime_seconds;
    strlcpy(entry.message, message, sizeof(entry.message));
    uint16_t used = 0;
    for (size_t i = 0; i < g_ring.count; ++i) {
        if (g_ring.slots[i] < FAULT_RING_SLOTS) used |= 1U << g_ring.slots[i];
    }
    uint8_t slot = 0;
    while (used & (1U << slot)) ++slot;
    fault_ring_metadata_t next = g_ring;
    if (next.count == M45_FAULT_LOG_CAPACITY) {
        memmove(next.slots, next.slots + 1, M45_FAULT_LOG_CAPACITY - 1);
        --next.count;
    }
    next.slots[next.count++] = slot;
    next.next_id = entry.id == UINT32_MAX ? 1 : entry.id + 1;
    err = write_ring_entry(nvs, slot, &entry);
    if (err == ESP_OK) err = publish_ring(nvs, &next);
    if (err == ESP_OK) cleanup_legacy_slots(nvs);
    nvs_close(nvs);
    if (err == ESP_OK) {
        if (g_faults.count == M45_FAULT_LOG_CAPACITY) {
            memmove(g_faults.entries, g_faults.entries + 1,
                    sizeof(g_faults.entries[0]) * (M45_FAULT_LOG_CAPACITY - 1));
            --g_faults.count;
        }
        g_faults.entries[g_faults.count++] = entry;
        g_faults.next_id = next.next_id;
        g_last_recorded_id_this_boot = entry.id;
    }
    pthread_mutex_unlock(&g_fault_lock);
    return err;
}

size_t m45_fault_log_snapshot(m45_fault_entry_t *entries, size_t capacity)
{
    if (entries == NULL || capacity == 0) {
        return 0;
    }
    pthread_mutex_lock(&g_fault_lock);
    if (!g_loaded && fault_store_load_locked() != ESP_OK) {
        pthread_mutex_unlock(&g_fault_lock);
        return 0;
    }
    const size_t count = g_faults.count < capacity ? g_faults.count : capacity;
    for (size_t i = 0; i < count; ++i) {
        entries[i] = g_faults.entries[g_faults.count - 1U - i];
    }
    pthread_mutex_unlock(&g_fault_lock);
    return count;
}

esp_err_t m45_fault_log_remove(uint32_t id)
{
    pthread_mutex_lock(&g_fault_lock);
    size_t index = g_faults.count;
    for (size_t i = 0; i < g_faults.count; ++i) {
        if (g_faults.entries[i].id == id) {
            index = i;
            break;
        }
    }
    if (index == g_faults.count) {
        pthread_mutex_unlock(&g_fault_lock);
        return ESP_ERR_NOT_FOUND;
    }
    nvs_handle_t nvs;
    esp_err_t err = open_ring_locked(&nvs);
    if (err != ESP_OK) {
        pthread_mutex_unlock(&g_fault_lock);
        return err;
    }
    fault_ring_metadata_t next = g_ring;
    memmove(next.slots + index, next.slots + index + 1, next.count - index - 1);
    next.slots[--next.count] = 0;
    err = publish_ring(nvs, &next);
    if (err == ESP_OK) cleanup_legacy_slots(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) {
        pthread_mutex_unlock(&g_fault_lock);
        return err;
    }
    if (index + 1U < g_faults.count) {
        memmove(&g_faults.entries[index], &g_faults.entries[index + 1U],
                sizeof(g_faults.entries[0]) * (g_faults.count - index - 1U));
    }
    memset(&g_faults.entries[g_faults.count - 1U], 0, sizeof(g_faults.entries[0]));
    --g_faults.count;
    pthread_mutex_unlock(&g_fault_lock);
    return err;
}

esp_err_t m45_fault_log_clear(void)
{
    pthread_mutex_lock(&g_fault_lock);
    nvs_handle_t nvs;
    // An explicit clear may replace unreadable history; ordinary appends must
    // first restore it and must never publish an empty fallback automatically.
    esp_err_t err = nvs_open(FAULT_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        const fault_ring_metadata_t next = {
            .magic = FAULT_RING_MAGIC,
            .next_id = g_loaded ? g_faults.next_id : 1,
        };
        err = publish_ring(nvs, &next);
        if (err == ESP_OK) cleanup_legacy_slots(nvs);
        nvs_close(nvs);
        if (err == ESP_OK) {
            fault_store_reset();
            g_faults.next_id = next.next_id;
            g_last_recorded_id_this_boot = 0;
            g_loaded = true;
        }
    }
    pthread_mutex_unlock(&g_fault_lock);
    return err;
}
