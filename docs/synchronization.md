# Timing across the four sensors

The goal is to compare when a pulse reached each sensor. The firmware does not force coincident peaks or remove real propagation delays. Each sensor retains its independent sample clock.

## Current acquisition: direct INT1 reads, version 5

The user confirmed sensor INT1 A/B/C/D on ESP32 GPIO20/21/22/23. New standalone recordings enable pulsed high-g data-ready (`CTRL4=0x02`, `CTRL7=0x80`) and high-g register output (`CTRL1_XL_HG=0xBC`, including bit 7). After the HAODR engines settle, FIFO is bypassed. Each positive GPIO interrupt captures the common ESP32 microsecond timer and immediately reads all six XYZ bytes at 10 MHz SPI. Sensors keep independent sampling rates and sample edges. There is no rate normalization, sample insertion or per-channel peak shift.

SPI2 is acquired exclusively for the direct session. The priority-3 GPIO ISR, timer call, register read routine and queue storage reside in internal memory so flash writes can proceed with caches unavailable. Reads use the peripheral register buffer without DMA or normal driver calls from the ISR, and have a bounded 40 us timeout. Each sensor has a 512-entry RAM ring; the task writes the frames to previously erased flash. Producer sequence, queue backlog, invalid reads and data-ready intervals over 200 us are checked. Errors stop acquisition and flag the retained data; missing XYZ values are never invented. Start/stop paths disable the producer before changing SPI configuration. The earlier live USB/TCP/browser streams continue to use FIFO framing.

Version 5 reuses the `IM4S` row encoding with a different, explicitly versioned time unit: the absolute counter and deltas are ESP32 microseconds since the common recording origin. A delta over 255 us starts another frame. Decoders use these original values directly, without a sensor-clock fit. NPZ exports retain `imuN_drdy_time_us` and `imuN_time_s`; the latter applies one common trigger-origin shift to every sensor. Older versions keep their original interpretation and remain readable.

These are interrupt-entry timestamps, not hardware edge captures. Interrupt scheduling, ordering of simultaneous edges, sensor filtering and mechanical propagation can affect measured pulse timing. The timer's 1 us resolution is not an accuracy bound. A common-impact test is still needed for a physical pairwise mismatch bound.

### Connected-board results, 6 October 2026

A two-minute RAM-only direct test acquired 3,691,418 samples. Each channel's count matched its data-ready edge count, with no late interval, SPI error or queue overflow. Valid stationary XYZ values were verified after correcting the high-g register-output enable bit. Individual reads took at most 10 us.

Two full-capacity flash recordings lasted 29.897276 s and 29.897624 s, retaining 919,667 and 919,665 samples respectively. Both had quality=0, saturation=0, no timestamp discontinuities and no coverage warnings. The repeat's edge and stored-sample counts matched exactly on every channel; maximum queue backlog was 98 of 512 entries. Observed intervals across these captures were 89-174 us; this range includes ISR scheduling jitter and is not a pairwise alignment measurement. Individual sensor rates were approximately 7685.4, 7689.4, 7692.1 and 7694.6 Hz. All original times remain intact.

The first full recording's host/browser exports match every raw count and original timestamp exactly, and common-axis time arrays agree to 1e-12 s. Manual 1+1 s triggering passed without quality errors. A stationary-gravity 0.5 g automatic trigger retained its triggering sample at exactly zero; only the expected incomplete pre-history flag was present. No physical common-impact test has been performed.

The final installed firmware's near-capacity manual-trigger run acquired another 919,672 samples with edge/sample counts equal on every channel, no late data-ready interval and maximum SPI read time 10 us. Its only quality flag was the expected incomplete post-window at capacity. Native timestamps and raw counts were preserved through reset and matched host/browser exports.

### Investigation of the earlier FIFO intervals

The FIFO status BDU read order was corrected: status 1 and 2 are read together, and overrun checks use that latched result. Rare FIFO timestamp gaps persisted in RAM-only tests without flash writes. Experiments with gyro enabled, clock reads removed, retained FIFO words, BDU disabled, high-g alone, faster SPI and smaller FIFO bursts did not reliably eliminate them; those configuration experiments were reverted. FIFO timestamp anomalies are not automatically proof of a missing physical conversion. Direct acquisition avoids the affected FIFO path and currently has no observed missed data-ready sample in the tests above.

A final 20 s FIFO/DRDY audit used the normal FIFO configuration (`CTRL1_XL_HG=0x3C`) with INT1 routed and unused register output disabled. It still found one unusual FIFO interval on A, five on C (including two duplicate-then-long timestamp pairs), and one on D. Corresponding DRDY intervals were 129-131 us, with no late DRDY interval, skipped tag or unmatched FIFO pair. The timestamp anomaly therefore does not require a delayed data-ready edge. An earlier audit with register output enabled but not consumed also produced tag/pair corruption and is excluded from this normal-mode comparison. Digital output timing and analog conversion timing are not interchangeable measurements.

## Legacy version 3/4 shared start

SPI chip selects use GPIO control. Every normal transaction selects one sensor, while the bus is acquired exclusively. After individually preparing the sensors, a single atomic GPIO mask selects every detected sensor for a write-only broadcast. The timestamp counters reset together (`TIMESTAMP2=0xAA`), followed by the required 400 us wait. One broadcast enables the FIFOs with a timestamp at every time slot (`FIFO_CTRL4=0x46`). The common FIFO-enable CS release is recorded with the ESP32 timer, and each sensor's configuration is read back individually. Broadcast reads are prohibited because the MISO outputs would contend. Existing live streams use the shared FIFO start without timestamp batching so their binary format remains unchanged.

The sensors run in high-accuracy ODR (HAODR) mode. Both the low-g engine and powered-down gyro select that mode, which also controls high-g sampling. Low-g data is not batched. Startup waits at least 500 us after power-down and 75 ms for the gyro-off clock to settle before resetting timestamps and enabling the FIFOs. Enabling the FIFOs together establishes a common recording boundary, not coincident sample edges.

## Legacy version 3/4 hardware time axis

The LSM6DSV320X timestamp counter runs nominally at 46,080 ticks/s (21.7 us per tick). The timestamp clock and sample rate are characterized separately; normal HAODR sample intervals can span five or six sensor ticks because of timestamp quantization. Version 4 stores the actual FIFO timestamp for every XYZ sample, paired by the FIFO time-slot tag. Either word ordering and pairs split across SPI reads are supported. Counts exclude timestamp words. An unmatched data word, unexpected tag, or abnormal timestamp step flags the recording. The normal storage cost is one timestamp-difference byte per sample plus a four-byte absolute timestamp per frame; a difference above 255 ticks starts a new frame, preserving the full counter.

For standalone BVR recordings, each sensor's timestamp registers are also read at startup, every 250 ms and before stopping. The ESP32 time before and after each SPI read brackets the measurement; its midpoint, width and sensor counter are saved. The decoder maps each original sample counter to the ESP32 clock by interpolating between clock calibration points. A weighted global clock fit reports average rates and supports short extrapolation at the boundaries. Reads wider than the shortest bracket plus 50 us are excluded. Local clock interpolation follows slow oscillator drift. FIFO-read latency does not enter this time axis. The decoder uses one common trigger origin for all channels, preserving their relative timing. No resampling or per-channel peak shift is applied. Plots and NPZ exports retain explicit per-sample times; version 4 NPZ files also include each channel's original `imuN_sensor_ticks` counters.

Malformed timing, missing hardware timestamps, missing clock calibration, non-monotonic counters and backwards sample/tick progression are rejected. Extra intervals are retained at their exact sample boundary and flagged for review; the software never inserts a fabricated XYZ sample or closes the gap. There is no silent fallback to read-time alignment. Sensor timestamp rollover is rejected; the approximately 26-hour rollover cannot occur within the flash-limited capture duration after the shared reset. Legacy version 3 recordings still interpolate between their decimated anchors; the location of a gap within their 32-sample interval cannot be recovered. Version 1 and 2 captures retain their original software timing method.

## Legacy version 3/4 characterization

The browser and exports report `timing_diagnostics` per sensor: clock calibration count, FIFO timestamp count, fitted tick duration, global clock-fit RMS and maximum residual, local clock interpolation residual, maximum FIFO interval deviation (`fifo_fit_max_ticks`), discontinuity count, sample interval and an estimated mapping uncertainty. The local residual compares each interior clock reading against interpolation through its neighbors with that reading left out. The version 4 uncertainty estimate adds the maximum local residual, the largest accepted read half-width, and one timestamp tick. Legacy version 3 also adds its largest FIFO interval deviation because the exact gap position is unknown. This is a characterization metric, not a certified error bound; it excludes analog/filter delay, mounting and physical wave propagation. Estimated mapping uncertainty above 1 ms and any timestamp discontinuities flag the recording for review.

For physical acceptance, mount all sensors on a fixture with a characterized common excitation and record repeated impulses near the beginning, middle and end of a capture. Compare pulse timing on the common time axis, accounting for orientation and mechanical propagation. Record pairwise offsets, repeatability and change over the capture. A 50 g threshold crossing can differ between channels because their pulse amplitudes and shapes differ; forcing each channel's individual crossing to zero would hide that information.

### Version 4 connected-board results, 6 October 2026

The first full-capacity recording lasted 30.756271 s and retained 906,561 XYZ samples with exactly as many original hardware timestamps. Every adjacent timestamp step on all four sensors was six ticks; no unmatched pairs, timestamp gaps, quality errors or saturation were reported. Counts and original sensor counters match host/browser NPZ exports exactly; common-axis time arrays agree to 1e-10 s. Mapping uncertainty was 58–65 us per channel. This removes decimated timestamp association from new recordings; it does not establish the root cause of the legacy version 3 extra intervals or certify physical pulse skew.

### Earlier version 3 characterization

Follow-up: subsequent version 4 HP captures still showed rare 12-tick intervals. A diagnostic recorded `slot_step=1` during such an interval, so no skipped FIFO slot was observed. Running the supporting low-g engine faster did not eliminate it. HAODR mode improved rate agreement and one full capture was clear, but its repeat still showed one unusual interval each on C and D. The FIFO cause remains unresolved; version 4 solves exact timestamp retention and gap placement. Current version 5 bypasses that FIFO path as described above. Original counters are never rewritten to hide it.

A 29.876637 s full-capacity recording contained 119 accepted clock readings per sensor. Host and browser exports agreed on every hardware-derived sample time (absolute tolerance 1e-10 s) and every raw count.

| Sensor | Local clock interpolation residual, maximum | Estimated mapping uncertainty | Extra FIFO intervals |
| --- | --- | --- | --- |
| A | 18.50 us | 195.90 us | One interval with six extra ticks |
| B | 14.31 us | 191.64 us | One interval with six extra ticks |
| C | 21.87 us | 63.24 us | None |
| D | 15.23 us | 194.21 us | One interval with six extra ticks |

The extra intervals are approximately one sample period and flag that recording for review. A subsequent manual-trigger recording with a 1 s pre-window and 1 s post-window had no timing discontinuities or quality errors; its estimated mapping uncertainty was 54–56 us. Automatic triggering was also exercised using stationary gravity at a 0.5 g threshold. These checks characterize the digital time mapping, not pairwise physical pulse skew. No common-impact test has been performed. Hardware timestamp overhead reduced observed full-capacity duration from 32.49 s in a legacy recording to 29.88 s in the new format.

References: [ST AN6119, sections 6.4 and 9](https://www.st.com/resource/en/application_note/DM01076787.pdf), [ST SPI protocol](https://www.st.com/resource/en/datasheet/lsm6dsv320x.pdf).
