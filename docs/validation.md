# Validation

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
