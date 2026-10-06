# Validation

## Direct acquisition and original shared-clock timestamps - 6 October 2026

New standalone BVR version 5 recordings bypass FIFO. Sensor INT1 A-D on GPIO20-23 drives priority-3, IRAM-resident GPIO handlers, which timestamp each sample on the ESP32 timer and read XYZ at 10 MHz SPI. Original native timestamps and independent rates are preserved in both decoders and NPZ exports. No resampling or channel-specific time shift is applied. SPI timeouts, overwritten RAM slots and data-ready intervals over 200 us stop and flag acquisition rather than creating replacement samples.

The corrected RAM-only two-minute test acquired 3,691,418 samples, with every sample count equal to its corresponding interrupt count, no late intervals or read errors, and maximum SPI read time 10 us. Two full flash sessions lasted 29.897276 s and 29.897624 s, retaining 919,667 and 919,665 samples. Both had quality=0, saturation=0, no coverage warning and no timing discontinuities. The second session matched all four edge/sample counts exactly and reached maximum queue backlog 98 of 512. Measured intervals were 89-174 us across these flash tests; this includes interrupt scheduling jitter and is not a certified physical mismatch bound.

Host/browser exports of the first full session agree on every raw XYZ count and native timestamp, and all common-axis times to 1e-12 s. Manual 1+1 s triggering passed without quality errors. Stationary-gravity automatic triggering at 0.5 g placed the recorded threshold sample at exactly zero and reported only the expected missing pre-history flag. Legacy USB streaming passed before and after a direct session, without a reset between modes.

The final installed build also recorded 29.898292 s with a manual trigger near capacity. Stored counts were 229772/229897/229974/230029 and matched all four interrupt counts exactly; no late interval or SPI error occurred, and maximum queue backlog was 99. Only the expected capacity-limited post-window flag (`0x40`) was present. The BVR version 5 manifest remained recognized and downloadable after a hard reset, and host/browser exports of its retained window match original counts/timestamps exactly and common-axis times to 1e-12 s.

All 42 host/browser tests passed, including unchanged native jitter, independent sampling rates, retention of a real 5 ms pulse delay, common trigger origin, exact host/browser NPZ agreement, native gap placement and rejection of missing/backwards times. JavaScript syntax and diff whitespace checks passed. ESP-IDF 6.0.1 build passed: application 1,021,344 bytes (`0xf95a0`), leaving 27,232 bytes in the 1 MiB app partition. ELF symbols confirm the GPIO ISR, direct register-read routine and timer call reside in `.iram0.text`. The application was flashed at `0x10000` with hash verification; the partition layout is unchanged.

The FIFO status BDU read-order bug was fixed, but rare FIFO timestamp gaps persisted in non-writing RAM tests. Faster SPI, altered HAODR/gyro setup and smaller FIFO bursts did not reliably remove them. New direct acquisition avoids that path and has no detected missed data-ready sample in the tests above. This is observed digital acquisition performance; physical common-impact skew, analog/filter delays and behavior under heavy Wi-Fi traffic remain unmeasured. Test recordings, logs and the original partition backup are retained in `test-artifacts/`.

After all device tests, the complete original recording partition was restored at `0x110000` with esptool hash verification. Its download after reset is byte-identical to the pre-test backup: ID `36a5bae347fa8f95b0142a9277496d63`, SHA-256 `d0fa0b43f44d94ed9cb27c7d41f98fce25194267f0fe44ab5219ea2e56424c4a`. Final status is saved, error=0, quality=0, saturation=0. The new application remains installed.

## Per-sample hardware timestamps - 6 October 2026

BVR version 4 retains an original FIFO timestamp for every XYZ sample, encoded with one delta byte per sample and one absolute counter per frame. Both NPZ exporters preserve original uint32 sensor counters and explicit common-axis times. No channel is resampled or independently shifted. FIFO time-slot pairing survives split reads and either word order; unmatched data, slot skips and unusual timestamp intervals flag the recording. Large intervals start another frame without truncating the counter.

All 39 host/browser tests passed, covering independent rates, delayed drains, real channel delays, exact placement of timing gaps, quantized five/six-tick intervals, original-counter exports and legacy versions. ESP-IDF 6.0.1 build passed: image 1,010,912 bytes (`0xf6ce0`), with 37,664 bytes free in the 1 MiB partition. JavaScript syntax and whitespace checks passed. The application was installed with hash verification.

Repeated high-performance captures revealed rare 12-tick intervals despite FIFO slot counters advancing by one. That evidence argues against dropping a FIFO word in the host read but does not prove whether the sample edge was delayed or the internal timestamp advanced. Increasing the supporting low-g rate did not remove the interval. High-accuracy ODR mode, with the documented gyro-off 75 ms settling delay and 600 us power-down wait, reduces rate differences: measured channels were approximately 7,686-7,695 Hz. The first HAODR full-capacity recording lasted 29.853349 s with no errors; the repeat still flagged one interval each on C and D. The root cause is unresolved; no claim that all gaps have been eliminated is made. Original counts/timestamps remain available for diagnosis.

The final 8 MHz SPI full capture lasted 29.915176 s, with 920,217 retained XYZ samples and their original counters. It still flagged one interval each on B and D; reduced SPI speed did not eliminate the issue. Mapping uncertainty was 56-64 us per channel. The stationary-gravity automatic-trigger test passed with no quality errors; the manual 1+1 s window retained an irregular interval on A and was correctly flagged. The original one-second four-channel USB stream passed at the new rates. Final host/browser exports agree exactly on all counts and sensor counters, and to 1e-10 s on all common-axis times, including flagged intervals. The original recording `36a5bae347fa8f95b0142a9277496d63` was restored and its post-reset download is byte-identical (SHA-256 `d0fa0b43f44d94ed9cb27c7d41f98fce25194267f0fe44ab5219ea2e56424c4a`). Device test files, diagnostic logs and backups are retained in `test-artifacts/`. Physical common-impact skew is still unverified.

## Shared start and hardware timing — 6 October 2026

All four sensors now receive a broadcast timestamp reset and FIFO-enable write. Standalone recordings store FIFO hardware timestamp anchors and repeated ESP32/sensor clock measurements; browser plots and host/browser exports use explicit per-sample times. Timestamp discontinuities preserve their interval and flag the recording for review. The existing live-stream format remains compatible.

ESP-IDF 6.0.1 build passed: application 1,009,904 bytes (`0xf68f0`), leaving 38,672 bytes in the 1 MiB partition. All 30 host/browser tests passed, including synthetic common pulses with independent sensor rates and deliberately delayed FIFO reads, preservation of a real 5 ms channel offset, malformed hardware timing, flagged timing gaps and explicit-time plot navigation. JavaScript syntax and diff whitespace checks passed. The final application was flashed at `0x10000` on COM92 with hash verification.

Connected-board checks included stationary-gravity automatic triggering, a 29.876637 s full-capacity capture, a final manual trigger retaining 1 s before and after the event, and the original one-second four-channel USB stream. Manual capture quality/saturation/error were zero, with 54–56 us estimated clock-mapping uncertainty. Full-capacity host/browser exports agreed on all counts and sample times to 1e-10 s. Its estimated mapping uncertainty was 63–196 us; one extra six-tick interval on each of A, B and D was flagged for review despite a clear hardware-overrun mask. See [synchronization.md](synchronization.md) for characterization and the physical common-impact acceptance test still required.

The user's original version 2 recording was backed up before device tests and restored after installation; test recordings and restore images are retained in `test-artifacts/`. Its post-reset download is byte-identical to the backup: ID `36a5bae347fa8f95b0142a9277496d63`, SHA-256 `d0fa0b43f44d94ed9cb27c7d41f98fce25194267f0fe44ab5219ea2e56424c4a`. Restored status has error=0, quality=0 and saturation=0. These digital timing checks do not certify physical pulse skew or coincident sampling edges.

## Acquisition capacity update — 6 October 2026

The application partition is reduced to 1 MiB, the smallest 64 KiB-aligned size that fits this build. Recording storage grows to `0x6f0000` bytes (6.9375 MiB), ending at the 8 MiB flash boundary. Standalone acquisition now stops at storage capacity instead of 20 seconds, reserving 16 pages for the FIFO tail and trigger record. A capacity-limited post-window receives the existing incomplete flag. The browser shows elapsed time and storage utilization.

ESP-IDF 6.0.1 build and partition checks passed: application size 1,000,336 bytes (`0xf4390`), leaving 48,240 bytes in the 1 MiB application partition. All 24 host/browser tests passed, including capacity status at 31 seconds with a working manual trigger and 50% storage progress. JavaScript syntax and diff whitespace checks passed. Physical full-capacity acquisition and late-trigger tail behavior remain untested. This update has not been flashed. The changed recording offset requires backing up/migrating existing recordings before installation.

Installation follow-up: flashed the bootloader, expanded partition table and application on COM92 with esptool hash verification. Before flashing, saved recording `592173a25193e274ffd1966d4f1ba545` was downloaded and validated, and the old partition table was backed up. The recording was migrated to `0x110000`, updating only its manifest page capacity and CRC. The post-reset download exactly matches the expected migrated BVR; every raw page and all acquisition metadata are unchanged. Migrated BVR SHA-256: `4bb9531c7eaf209e4f193279372aa02bc365aaeb418dd043dd603a055f92a5b8`. Backups and migration images are in `test-artifacts/`. All four sensors responded with WHO_AM_I `0x73`; recorder status is saved with error=0, quality=0 and saturation=0. No new acquisition was started, and full-capacity duration remains untested.

## Earlier validation — 5 October 2026

## Connection recovery follow-up — installed on the board

The user reported a page stuck on Connecting. USB checks showed four healthy sensors and a saved recording; the PC could not reach the board HTTP address, so the original phone-side failure was not reproduced. Status requests now time out after five seconds, including stalled response bodies, release the polling lock and retry. An enabled Retry connection control and independent inline startup diagnostics expose missing/failed scripts instead of leaving an inert page. Versioned script URLs bypass stale caches; the HTTP handler correctly serves JavaScript when a query string is present.

All five browser regression tests passed, including new timeout/recovery and startup-error cases. JavaScript syntax, diff whitespace and ESP-IDF 6.0.1 build checks passed. The 999,872-byte application was flashed at `0x10000` on COM92 and verified by esptool. No partition erase or saved-recording deletion was performed. After reset all four sensors responded. Saved recording `7437173700296500d001dbc58184d68f` was downloaded before and after installation and is byte-identical (SHA-256 `5a256fa7211533286ab048529fd1a20eab378acb0ec875b14a2898a7d17bc221`); both backups are in `test-artifacts/`. Phone reload and HTTP interaction remain to be confirmed by the user.

## Threshold timing and plot interaction update

Automatic recording now latches the first observed sample at or above the configured XYZ magnitude and writes its sensor, absolute sample index and threshold into a BVR v2 event record. Browser and host decoders anchor that sample to time zero after fitting the sensor clocks. The firmware's post-window estimate accounts for later rows in the current FIFO batch and the remaining queued rows. Version 1 recordings remain readable with their original timing.

The responsive recording page now includes pinch/trackpad zoom, drag-to-pan, Shift+mouse-drag range selection, event focus on load, Center event / Full view controls, sensor visibility switches, a magnitude view with the stored threshold, keyboard navigation and readable canvas sizing on narrow phones. PNG export includes sensor labels.

Validation: all 22 host/browser tests passed, including threshold alignment under clock drift and delayed FIFO timestamps, host/browser NPZ agreement, legacy/manual/no-trigger cases, zero pre- or post-window, marker page boundaries and malformed event rejection. Interaction tests exercise two-pointer pinch, lifting one finger into a pan, cancellation, trackpad zoom anchors, zoom limits, range selection, event centering, narrow canvas coordinates, HTML control wiring, magnitude threshold labels and disconnected controls. JavaScript syntax and diff whitespace checks passed. ESP-IDF 6.0.1 build passed; final application is 997,184 bytes (`0xf3740`), with 52% of its 2 MiB partition free.

Browser automation was unavailable (no browser surface; in-app browser creation rejected), so rendered visual QA and physical phone gestures have not been verified. This update has not been flashed or tested against a physical impulse. The acquired threshold sample is exact in the exported sample timeline; analog crossing time and synchronization between independent sensors remain estimated.

## Unified recording UI update

The subsequent UI update uses one Start recording flow, defaults to 50 g and 10+3 s, accepts early pulses with an incomplete-history warning, and saves the full 20 s when no pulse occurs. The browser automatically plots stored recordings and supports wheel/button zoom, time-range selection, pan, full-view reset, raw BVR/NPZ download and PNG export.

Validation for this update: ESP-IDF 6.0.1 build passed (974,048-byte application, 54% partition free); 16 host/browser tests passed. Tests cover a full 20 s no-trigger session and export duration, early-pulse history, browser/host decoder agreement, NPZ counts and timing, corruption/truncation rejection, capture settings, zoom, pan and reset. JavaScript syntax and diff whitespace checks passed. Browser control was unavailable, so rendered visual QA and physical device tests of this update have not been performed. The build has not been flashed during this update.

## Earlier device validation (before the UI update)

The timeout and early-trigger results below describe the earlier firmware; they have been superseded by the behavior above.

The firmware was built and installed on the connected ESP32-C6FH8 revision v0.2 at COM92. All four LSM6DSV320X sensors responded with WHO_AM_I `0x73`. ESP-IDF version: 6.0.1. Final application: 963,248 bytes in a 2,097,152-byte partition; 54% remains free.

| Check | Result |
| --- | --- |
| Host binary integrity and export tests | 11 tests passed: CRC errors in manifest/header/payload, truncation, trailing bytes, reordered pages, sensor index gaps, window boundaries, timing independent of trim predictions, calibration orientation validation, NPZ inspector compatibility, analysis/comparison statistics and overwrite protection. |
| Final firmware build | Passed, including partition size checks. |
| Original USB streaming | Four-channel one-second live capture succeeded with frame/footer consistency and reopened in the existing inspector. |
| Full triggered window | 10 s pre-trigger + 3 s post-trigger; software trigger at 19.026977 s after arming. Quality mask=0, saturation mask=0; all four channels cover the estimated requested window. |
| Counts in final selected window | A=95,753; B=95,804; C=96,877; D=94,822. Independent sensor rates explain differing counts; nominal ODR remains 7,680 Hz. |
| Fitted rates | A=7,365.64; B=7,369.55; C=7,452.06; D=7,294.06 Hz. These are estimated from drain times, not proof of synchronized sample edges. |
| Persistence | Final recording ID `34a07fb27943af31ceecbd03b32c9b16` survived hard reset/application update. Download SHA-256 remained `de1ebb2085225f7191607f75d77500b1c5c8f3d28c3881bcb9a2704cfffe0910`. The recording remains on the board. |
| Waiting timeout | After 20 s without a trigger: state=error, saved=0, error=263 (ESP_ERR_TIMEOUT). An expired trigger was rejected. Rearming then succeeded. |
| Early trigger | Rejected before the requested pre-window was accumulated. |
| Automatic trigger | 200 ms pre + 500 ms post, threshold 0.5 g against stationary gravity; committed a four-channel capture with no reported errors. This verifies the threshold path, not release detection in real archery. |
| Cancel during post-window | Saved for diagnosis with incomplete flag `0x40`, rejected as valid by the host decoder. |
| Overwrite protection | A new arm request was rejected while a shot was retained. Explicit deletion was required. |
| Exports/analysis | Real maximum-window recording exported to raw/processed NPZ, CSV, JSON and analysis PNG. Host tests also verified comparison statistics and exclusion of flagged shots. |
| JavaScript syntax / diff whitespace | Passed. |

The rejected continuous-ring experiment used 4 KiB sector erases during acquisition and reported overrun mask `0x0f`. It is not the final acquisition design. The user approved the final prepared-flash, bounded-wait approach; the final recorder does not erase while sampling. Auto-suspend force-enable flags remain disabled for this chip revision.

Remaining acceptance checks: physical power-off/on persistence (hard reset was tested), phone HTTP/WebSocket operation while connected to BowVib Wi-Fi, raw TCP streaming after the upgrade, common-impulse synchronization/skew, calibrated high-g range/saturation, repeatability over 100 real shots and mechanical mounting safety. The PC was not connected to the board AP during this session, so HTTP status could not be reached; no phone/browser end-to-end test is claimed.

The added USB shot commands were tested locally. A new raw-download command on TCP was omitted following automatic approval review; browser HTTP download remains implemented. Customer hardware gaps and contractual deliverables are tracked in `customer-requirements.md`.
