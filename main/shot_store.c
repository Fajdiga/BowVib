#include "shot_store.h"
#include "lsm6dsv320x.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "esp_rom_crc.h"
#include <string.h>

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t seq, used, crc;
    uint64_t first_us, last_us;
} page_header_t;
_Static_assert(sizeof(page_header_t) == 32, "disk header size");
_Static_assert(sizeof(shot_manifest_t) == 608, "disk manifest size");
static const esp_partition_t *s_partition;
static shot_manifest_t s_manifest;
static uint8_t s_page[SHOT_PAGE_BYTES];
static volatile uint32_t s_seq;
static uint32_t s_pages;
static bool s_saved, s_writing, s_corrupt;
static bool s_trigger_marked;
static uint32_t s_trigger_data_seq;

bool shot_store_available(void) { return s_partition != NULL; }
uint32_t shot_store_capacity_bytes(void) { return s_pages * SHOT_PAGE_BYTES; }
uint32_t shot_store_used_bytes(void)
{
    return (s_writing ? s_seq + 1U : s_saved ? s_manifest.end_seq : 0U) * SHOT_PAGE_BYTES;
}
bool shot_store_near_capacity(void)
{
    return s_writing && s_seq + 1U + SHOT_TAIL_RESERVE_PAGES >= s_pages;
}
bool shot_store_saved(void) { return s_saved; }
bool shot_store_corrupt(void) { return s_corrupt; }
const shot_manifest_t *shot_store_manifest(void) { return &s_manifest; }
static size_t page_offset(uint32_t seq) { return SHOT_PAGE_BYTES * (1U + seq % s_pages); }

esp_err_t shot_store_init(void)
{
    s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, "shots");
    if (!s_partition || s_partition->size < (SHOT_TAIL_RESERVE_PAGES + 2U) * SHOT_PAGE_BYTES) return ESP_ERR_NOT_FOUND;
    s_pages = s_partition->size / SHOT_PAGE_BYTES - 1U;
    esp_err_t err = esp_partition_read(s_partition, 0, &s_manifest, sizeof(s_manifest));
    if (err != ESP_OK) return err;
    s_saved = !memcmp(s_manifest.magic, "BVR1", 4) &&
        (s_manifest.version >= 1 && s_manifest.version <= 5) &&
        s_manifest.page_count == s_pages && s_manifest.end_seq >= s_manifest.first_seq &&
        s_manifest.end_seq - s_manifest.first_seq <= s_pages &&
        s_manifest.ended_us >= s_manifest.started_us &&
        (!s_manifest.trigger_us || (s_manifest.trigger_us >= s_manifest.started_us && s_manifest.ended_us >= s_manifest.trigger_us)) &&
        s_manifest.crc == esp_rom_crc32_le(0, (const uint8_t *)&s_manifest, sizeof(s_manifest) - 4);
    bool erased = true;
    for (size_t i = 0; i < sizeof(s_manifest); ++i)
        if (((const uint8_t *)&s_manifest)[i] != 0xFF) { erased = false; break; }
    s_corrupt = !s_saved && !erased;
    return ESP_OK;
}

esp_err_t shot_store_delete(void)
{
    if (!s_partition || s_writing) return ESP_ERR_INVALID_STATE;
    esp_err_t err = esp_partition_erase_range(s_partition, 0, SHOT_PAGE_BYTES);
    if (err == ESP_OK) { s_saved = false; s_corrupt = false; }
    return err;
}

esp_err_t shot_store_begin(uint32_t pre_ms, uint32_t post_ms, const char *metadata)
{
    if (!s_partition || s_saved || s_corrupt || s_writing) return ESP_ERR_INVALID_STATE;
    if (pre_ms > 10000 || post_ms > 3000 || pre_ms + post_ms == 0 || !metadata || strlen(metadata) >= SHOT_METADATA_BYTES)
        return ESP_ERR_INVALID_ARG;
    /* Erase before enabling IMUs; never erase a saved shot implicitly. */
    esp_err_t err = esp_partition_erase_range(s_partition, 0, s_partition->size);
    if (err != ESP_OK) return err;
    memset(&s_manifest, 0, sizeof(s_manifest));
    memcpy(s_manifest.magic, "BVR1", 4);
    s_manifest.version = 5;
    s_manifest.page_count = s_pages;
    s_manifest.pre_ms = pre_ms; s_manifest.post_ms = post_ms;
    s_manifest.odr_hz = LSM6DSV320X_ODR_HZ; s_manifest.fs_g = LSM6DSV320X_FS_G;
    s_manifest.present_mask = lsm6dsv320x_present_mask();
    for (unsigned i = 0; i < 4; ++i) s_manifest.freq_fine[i] = lsm6dsv320x_internal_freq_fine(i);
    esp_fill_random(s_manifest.id, sizeof(s_manifest.id));
    memcpy(s_manifest.metadata, metadata, strlen(metadata) + 1);
    memset(s_page, 0xFF, sizeof(s_page));
    ((page_header_t *)s_page)->used = sizeof(page_header_t);
    s_seq = 0; s_writing = true; s_trigger_marked = false;
    return ESP_OK;
}

static esp_err_t flush_page(void)
{
    page_header_t *h = (page_header_t *)s_page;
    if (h->used == sizeof(*h)) return ESP_OK;
    /* This board cannot erase during acquisition without FIFO overruns.
       Exhaustion stops the recording, never overwrites earlier raw data. */
    if (s_seq >= s_pages) return ESP_ERR_NO_MEM;
    memcpy(h->magic, "BVP1", 4); h->seq = s_seq;
    h->crc = 0;
    h->crc = esp_rom_crc32_le(0, s_page, h->used);
    const size_t offset = page_offset(s_seq);
    esp_err_t err = ESP_OK;
    if (err == ESP_OK) err = esp_partition_write(s_partition, offset, s_page, SHOT_PAGE_BYTES);
    if (err != ESP_OK) return err;
    ++s_seq;
    memset(s_page, 0xFF, sizeof(s_page));
    ((page_header_t *)s_page)->used = sizeof(page_header_t);
    return ESP_OK;
}

esp_err_t shot_store_append(const uint8_t *frame, size_t len, uint64_t time_us, uint32_t sample_index)
{
    if (!s_writing || len < 7 || len + 12 > SHOT_PAGE_BYTES - sizeof(page_header_t)) return ESP_ERR_INVALID_STATE;
    page_header_t *h = (page_header_t *)s_page;
    if (h->used + 12 + len > SHOT_PAGE_BYTES) {
        esp_err_t err = flush_page();
        if (err != ESP_OK) return err;
    }
    if (h->used == sizeof(*h)) h->first_us = time_us;
    h->last_us = time_us;
    memcpy(s_page + h->used, &time_us, 8);
    memcpy(s_page + h->used + 8, &sample_index, 4);
    memcpy(s_page + h->used + 12, frame, len);
    h->used += 12 + len;
    return ESP_OK;
}

esp_err_t shot_store_timestamp(uint64_t drain_us, uint8_t sensor, uint32_t index, uint32_t ticks)
{
    uint8_t event[11] = {'I', 'M', '4', 'H', sensor, 0, 0};
    memcpy(event + 7, &ticks, 4);
    return shot_store_append(event, sizeof(event), drain_us, index);
}

esp_err_t shot_store_clock(uint64_t drain_us, uint8_t sensor, uint32_t ticks,
                          uint64_t midpoint_us, uint32_t span_us)
{
    uint8_t event[23] = {'I', 'M', '4', 'C', sensor, 0, 0};
    memcpy(event + 7, &ticks, 4);
    memcpy(event + 11, &midpoint_us, 8);
    memcpy(event + 19, &span_us, 4);
    return shot_store_append(event, sizeof(event), drain_us, 0);
}

esp_err_t shot_store_mark_trigger(uint64_t time_us, uint8_t sensor,
                                  uint32_t sample_index, uint32_t threshold_mg)
{
    if (!s_writing || s_trigger_marked) return ESP_ERR_INVALID_STATE;
    if (sensor >= LSM6DSV320X_COUNT || threshold_mg < 20U || threshold_mg > 320000U)
        return ESP_ERR_INVALID_ARG;
    /* Same record prefix as IM4D, with zero XYZ rows and a uint32 threshold.
       Keep the data page even when the event record starts a new flash page. */
    uint8_t event[11] = {'I', 'M', '4', 'T', sensor, 0, 0};
    memcpy(event + 7, &threshold_mg, sizeof(threshold_mg));
    const uint32_t data_seq = s_seq;
    const esp_err_t err = shot_store_append(event, sizeof(event), time_us, sample_index);
    if (err == ESP_OK) {
        s_trigger_data_seq = data_seq;
        s_trigger_marked = true;
    }
    return err;
}

esp_err_t shot_store_read_page(uint32_t seq, void *page)
{
    if (!s_partition || !page) return ESP_ERR_INVALID_STATE;
    esp_err_t err = esp_partition_read(s_partition, page_offset(seq), page, SHOT_PAGE_BYTES);
    if (err != ESP_OK) return err;
    page_header_t *h = page;
    if (memcmp(h->magic, "BVP1", 4) || h->seq != seq || h->used <= sizeof(*h) || h->used > SHOT_PAGE_BYTES)
        return ESP_ERR_INVALID_CRC;
    const uint32_t stored_crc = h->crc;
    h->crc = 0;
    const uint32_t actual_crc = esp_rom_crc32_le(0, page, h->used);
    h->crc = stored_crc;
    if (actual_crc != stored_crc) return ESP_ERR_INVALID_CRC;
    return ESP_OK;
}

esp_err_t shot_store_finish(uint64_t started, uint64_t trigger, uint64_t ended,
                            uint8_t quality, uint8_t saturation, bool save_full_session)
{
    if (!s_writing) return ESP_ERR_INVALID_STATE;
    esp_err_t err = flush_page();
    s_writing = false;
    if (err != ESP_OK) return err;
    if (!trigger && !save_full_session) return ESP_OK; /* Explicit cancel: no committed shot. */
    s_manifest.started_us = started; s_manifest.trigger_us = trigger; s_manifest.ended_us = ended;
    s_manifest.quality_mask = quality; s_manifest.saturation_mask = saturation;
    s_manifest.end_seq = s_seq;
    s_manifest.first_seq = s_seq > s_pages ? s_seq - s_pages : 0;
    const uint64_t cutoff = !trigger ? started :
        (trigger > (uint64_t)s_manifest.pre_ms * 1000 ? trigger - (uint64_t)s_manifest.pre_ms * 1000 : 0);
    if (trigger && trigger - started < (uint64_t)s_manifest.pre_ms * 1000)
        s_manifest.quality_mask |= 0x20U; /* Early pulse: preserve available history. */
    /* Retain the page crossing the window boundary. Host trims individual records. */
    while (s_manifest.first_seq < s_seq) {
        if (s_trigger_marked && s_manifest.first_seq == s_trigger_data_seq) break;
        page_header_t h;
        err = esp_partition_read(s_partition, page_offset(s_manifest.first_seq), &h, sizeof(h));
        if (err != ESP_OK) return err;
        if (h.last_us >= cutoff) break;
        ++s_manifest.first_seq;
    }
    if (s_manifest.first_seq == s_seq) s_manifest.quality_mask |= 0x20U;
    else {
        page_header_t h;
        err = esp_partition_read(s_partition, page_offset(s_manifest.first_seq), &h, sizeof(h));
        if (err != ESP_OK) return err;
        /* One FIFO chunk of boundary tolerance; any larger loss is flagged. */
        if (h.first_us > cutoff + 40000U) s_manifest.quality_mask |= 0x20U;
    }
    s_manifest.crc = esp_rom_crc32_le(0, (const uint8_t *)&s_manifest, sizeof(s_manifest) - 4);
    /* Manifest written LAST: an interrupted recording never appears committed. */
    err = esp_partition_write(s_partition, 0, &s_manifest, sizeof(s_manifest));
    s_saved = err == ESP_OK;
    return err;
}
