#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Compare in micro-g squared: one count is 10,417 micro-g. Keeping the
   fractional count avoids triggering just below the requested threshold.
   Three full-scale int16 axes and 320,000 mg both fit in uint64_t. */
static inline bool capture_threshold_reached(uint64_t norm_counts_squared,
                                             uint32_t threshold_mg)
{
    const uint64_t threshold_ug = (uint64_t)threshold_mg * 1000U;
    return threshold_mg != 0U &&
        norm_counts_squared * UINT64_C(10417) * UINT64_C(10417) >=
        threshold_ug * threshold_ug;
}

/* Software estimate used to end the post-window. The persisted sample index,
   rather than this estimate, anchors the event in exported/plot time axes. */
static inline int64_t capture_sample_time_us(int64_t started_us, int64_t drain_us,
                                             uint32_t samples_after, uint32_t odr_hz,
                                             int8_t freq_fine)
{
    const int32_t fine = freq_fine == INT8_MIN ? 0 : freq_fine;
    const uint64_t rate_scaled = (uint64_t)odr_hz * (uint32_t)(10000 + 13 * fine);
    const int64_t age_us = (int64_t)((uint64_t)samples_after * UINT64_C(10000000000) / rate_scaled);
    const int64_t sample_us = drain_us - age_us;
    return sample_us > started_us ? sample_us : started_us;
}
