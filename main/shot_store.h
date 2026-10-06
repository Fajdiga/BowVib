#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define SHOT_PAGE_BYTES 4096U
#define SHOT_METADATA_BYTES 512U
/* Reserve space for the finite FIFO tail and trigger record. */
#define SHOT_TAIL_RESERVE_PAGES 16U
/* Disk structures are little-endian, packed, and versioned; see docs/recording.md. */
typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t version, first_seq, end_seq, page_count, pre_ms, post_ms;
    uint32_t odr_hz, fs_g, present_mask, quality_mask, saturation_mask;
    uint64_t started_us, trigger_us, ended_us;
    uint8_t id[16];
    int8_t freq_fine[4];
    char metadata[SHOT_METADATA_BYTES];
    uint32_t crc;
} shot_manifest_t;

esp_err_t shot_store_init(void);
bool shot_store_available(void);
uint32_t shot_store_capacity_bytes(void);
uint32_t shot_store_used_bytes(void);
bool shot_store_near_capacity(void);
bool shot_store_saved(void);
bool shot_store_corrupt(void);
esp_err_t shot_store_delete(void);
esp_err_t shot_store_begin(uint32_t pre_ms, uint32_t post_ms, const char *metadata);
esp_err_t shot_store_append(const uint8_t *frame, size_t len, uint64_t time_us, uint32_t sample_index);
/* Persist the precise threshold sample after its data frame (BVR v2-v4). */
esp_err_t shot_store_mark_trigger(uint64_t time_us, uint8_t sensor,
                                  uint32_t sample_index, uint32_t threshold_mg);
esp_err_t shot_store_timestamp(uint64_t drain_us, uint8_t sensor, uint32_t index, uint32_t ticks);
esp_err_t shot_store_clock(uint64_t drain_us, uint8_t sensor, uint32_t ticks,
                          uint64_t midpoint_us, uint32_t span_us);
esp_err_t shot_store_finish(uint64_t started, uint64_t trigger, uint64_t ended,
                            uint8_t quality, uint8_t saturation, bool save_full_session);
const shot_manifest_t *shot_store_manifest(void);
esp_err_t shot_store_read_page(uint32_t seq, void *page);
