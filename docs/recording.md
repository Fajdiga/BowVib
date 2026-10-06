# Recording on the ESP32-C6

The board page has one raw recording workflow. Press Start recording; data is written to internal flash, so the phone does not need to stay connected during acquisition. The USB, raw TCP and WebSocket streaming protocols remain available for existing clients.

## Why this uses prepared flash

Four channels at 7,680 Hz generate 184,320 raw XYZ bytes/s. A 10 s pre-trigger plus 3 s post-trigger window contains 2,396,160 raw bytes, excluding metadata/framing. This cannot fit in the C6's RAM.

The factory application partition is 1 MiB at `0x10000`; the `shots` partition is `0x6f0000` bytes (6.9375 MiB) at `0x110000`, ending at the 8 MiB flash boundary. Existing NVS/PHY offsets are unchanged. Download saved recordings before installing the changed partition table; recordings at the former `0x210000` offset need migration. Builds enforce the 1 MiB firmware limit. The recorder adds a 4 KiB staging page, a 608-byte manifest and four 512-entry direct-sample rings (48 KiB total); it keeps the existing 32 KiB streaming queue rather than allocating a multi-megabyte RAM buffer.

The first board test of repeated 4 KiB erase/write during acquisition reported FIFO overruns on every sensor. The connected chip is ESP32-C6FH8 revision v0.2 with flash ID `0x204017`. ESP-IDF 6.0.1 disables auto-suspend for affected C6 revisions; force-enable options are not used. See [Espressif flash concurrency](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32c6/api-reference/peripherals/spi_flash/spi_flash_concurrency.html) and the local SDK `components/spi_flash/Kconfig` for the revision restriction.

The entire recording partition is erased **before** sensors start. The recorder appends pages without erasing or overwriting during acquisition. The default automatic trigger is 50 g XYZ magnitude on any sensor, with 10 s before and 3 s after the pulse. Detection starts immediately; an early pulse retains all available history and flags the missing pre-window. Acquisition has no fixed 20 s deadline: without a pulse it continues until the data partition approaches capacity, then stops the sensors, drains the final RAM queue and commits the full session. One page holds the manifest; 16 data pages are reserved for the finite tail and trigger record. Actual duration depends on sensor count, oscillator trim, frame sizes and page packing. A pulse selects the trailing pre-window and requested post-window; reaching capacity early marks its tail incomplete (`0x40`). Unexpected write/exhaustion errors still report failure. Status includes `capacity_bytes`, `used_bytes` (allocated pages, including the staging page), and `elapsed_ms` instead of the previous fixed-time countdown.

The last shot is never automatically replaced. Download/export it, then explicitly delete it before arming again. Finite flash endurance still applies; arm for a shot rather than using the device as an unattended continuous logger. A multi-shot archive belongs on the phone/PC.

A non-erased manifest that fails validation is reported as `corrupt` and also blocks rearming until explicitly deleted. Incomplete uncommitted captures have no valid manifest and are never reported as successful shots.

## Phone and USB use

Connect to `BowVib-IMU`, open `http://192.168.4.1/`, and press **Start recording**. Threshold and pre/post settings default to 50 g and 10+3 s. Set 0 g for manual triggering using **Trigger now**. Optional JSON test notes are collapsed by default. Automatic triggering uses uncalibrated XYZ magnitude, including gravity; thresholds below resting magnitude trigger immediately.

After saving, the page automatically loads and validates the stored `.bvr` recording and plots the XYZ axes with a separate color for each sensor. **Save raw recording** downloads the original BVR; **Save NPZ** downloads raw counts, native per-sensor sample times and metadata for the existing inspector. The plot initially focuses a one-second view around the event, bounded by the available recording. Uncheck **Focus event on load** to start with the full recording instead. **Center event** returns to that view; **Full view** shows all saved samples. Two-finger pinch, trackpad pinch, scrolling and buttons zoom; dragging pans; Shift+mouse-drag selects a time range. The slider and horizontal trackpad scrolling also pan. With the chart focused, use +/− to zoom, arrow keys to pan, E to center the event, and Home for the full view. Sensor checkboxes hide individual traces; **XYZ magnitude** shows the actual saved automatic threshold and triggering sample for new recordings. PNG export retains sensor labels. Plot reduction preserves each bucket's extrema so narrow peaks remain visible. Pulse windows use pulse-relative time; full sessions use time from recording start. Closing the browser does not stop acquisition. Cancelling before a trigger saves nothing; cancelling during the post-window saves a shot marked incomplete. A nonzero quality/saturation mask requires review.

USB controls do not reset the board's DTR/RTS lines:

```powershell
python shot_tool.py --port COM92 status
python shot_tool.py --port COM92 arm --pre 10 --post 3 --metadata test.json
# Defaults: automatic 50 g trigger, 10+3 s window, record until capacity if no pulse.
# Optionally trigger manually while recording still has capacity.
python shot_tool.py --port COM92 trigger
python shot_tool.py --port COM92 download shot.bvr
python shot_tool.py export shot.bvr exports
python shot_tool.py analyze shot.bvr shot.png
python shot_tool.py --port COM92 delete
```

Omit `--port` to use the board HTTP interface; `--host` defaults to `192.168.4.1`. `test.json` must be a JSON object under 512 UTF-8 bytes. For example, record `archer`, `bow`, `configuration`, `arrow`, `sensor_positions`, `orientations`, `mounting` and `trigger_threshold_g`. USB without `--metadata` stores `source=USB`. The server stores supplied metadata as opaque text; malformed JSON received from third-party clients is preserved as `raw_text` by the decoder.

```powershell
python shot_tool.py compare comparison.png shot1.bvr shot2.bvr
python shot_tool.py export shot.bvr exports --calibration calibration.json
```

Output files are protected against replacement unless `--overwrite` is supplied. Export creates a full raw-count NPZ compatible with `inspect_raw.py`, a CSV with raw/processed XYZ, resultant, sample indices and timing, and JSON metadata. Analysis adds XYZ, resultant, FFT, RMS envelope and a statistics JSON file. Comparison groups repeated peak statistics by the complete test metadata object and excludes flagged shots; use identical metadata for repeat measurements of a configuration.

USB command forms are `SHOTARM pre_ms post_ms threshold_mg [JSON-object]`, `SHOTTRIGGER`, `SHOTSTATUS`, `SHOTGET`, `SHOTCANCEL`, `SHOTDELETE`. `SHOTGET` sends `SHOT DATA bytes=N\n`, followed by exactly N BVR bytes. It is separate from the original `START`/`STOP` IM4 stream. No new TCP shot-download command was added.

## Binary BVR format (versions 1–5)

All numbers are little-endian. A download contains a 608-byte manifest followed by 4,096-byte pages in logical sequence order. Version 2 adds a threshold event record; version 3 adds hardware timing records without changing those sizes. Version 4 stores a FIFO hardware timestamp with each sample. Version 5 stores native ESP32 data-ready interrupt timestamps and bypasses FIFO acquisition. The updated firmware and decoders accept versions 1 through 5; older decoders must be updated for new recordings. Physical flash page zero is reserved for the manifest. Unused page bytes are `0xff`; CRC covers the header and all used bytes, excluding that padding. The manifest is committed only after sampling stops and the final page is written. An interrupted uncommitted capture does not appear saved. A hard reset after a committed capture preserves it; physical power-off acceptance remains a separate device test.

Manifest Python structure: `<4s11I3Q16s4b512sI`.

| Field group | Contents |
| --- | --- |
| Magic | `BVR1` |
| Eleven uint32 values | version=1 through 5, first_seq inclusive, end_seq exclusive, physical page capacity, pre_ms, post_ms, ODR Hz, full-scale g, sensor presence mask, quality mask, saturation mask |
| Three uint64 values | acquisition start, estimated threshold sample time or manual software trigger (0 = full session without a pulse), acquisition end, in microseconds from the C6 timer |
| 16 bytes | Random unique shot ID |
| Four int8 values | IMU oscillator trim, −128 means unavailable |
| 512 bytes | NUL-terminated UTF-8 metadata, zero-filled remainder |
| uint32 | Standard CRC32 of all preceding manifest bytes |

Page header Python structure: `<4sIIIQQ` (32 bytes): `BVP1`, logical sequence, total used bytes including header, CRC32, first record drain timestamp, last record drain timestamp. Calculate CRC32 over the used page with the CRC field at bytes 12–15 set to zero.

Each page contains whole records: uint64 drain timestamp, uint32 first absolute sensor sample index, then the unchanged IM4 data frame (`IM4D`, zero-based sensor uint8, XYZ row count uint16, 1–256 signed int16 XYZ rows). No record crosses a page boundary. The decoder checks header and data CRCs, page order, sensor validity, timestamp consistency and per-sensor index continuity before exporting. It rejects truncated or corrupt recordings.

A version 2 automatic trigger additionally writes exactly one 23-byte event record, Python structure `<QI4sBHI`: uint64 drain timestamp, uint32 absolute triggering sample index, `IM4T`, zero-based sensor uint8, uint16 zero, and uint32 threshold in mg. It follows the `IM4D` record containing that sample and carries the same drain timestamp, including when a page boundary falls between them. Page timestamp bounds include the event record. The recorder retains the triggering data page even with zero pre-trigger history. The decoder validates the event record and its sample reference. Version 2 recordings with a nonzero trigger time and no event record are manual triggers; `trigger_us=0` remains an untriggered full session. Version 1 recordings do not identify their trigger source.

Automatic detection latches the first acquired sample encountered at or above the requested XYZ magnitude, comparing squared magnitudes at the sensor's 10.417 mg/count sensitivity without rounding the threshold down to whole counts. The firmware stores its sensor and absolute sample index immediately instead of timestamping a request in the next capture-task iteration. Version 5 uses the sample's original ESP32 interrupt time for the post-window deadline. In version 3 and 4 it estimates the trigger time from FIFO ticks and the latest ESP32 clock calibration. Before the first hardware anchor, it uses the earlier drain-time estimate, subtracting later and queued samples using oscillator trim. Sensor queues are processed sequentially, so this selects the first observed crossing; it cannot establish which independent sensor crossed first in physical time.

Version 3 adds two zero-row record types using the same 19-byte record prefix. `IM4H` has a uint32 sensor timestamp tick count, for 23 bytes total; the prefix index identifies the high-g sample sharing its FIFO time-slot tag. `IM4C` has uint32 sensor ticks, uint64 ESP32 read midpoint in microseconds, and uint32 read bracket width in microseconds, for 35 bytes total; its prefix sample index is zero. The prefix time remains the append/drain time to preserve chronological page validation. The clock midpoint is separate and must lie within the acquisition and no later than its record time. Anchors may precede their referenced XYZ frame across a chunk/page boundary; only a final pending one-past-end anchor is excluded from fitting. The decoder maps indices to FIFO ticks and FIFO ticks to the common ESP32 clock instead of fitting drain times. Missing or inconsistent hardware timing is rejected. See [synchronization.md](synchronization.md) for fitting and diagnostic definitions.

Quality bits 0–3: sensor FIFO overrun/read/tag error in older formats, or direct SPI read/queue/data-ready interval error in version 5; bit 5: missing pre-window; bit 6: cancelled or capacity-limited post-window; bit 7: storage failure. Saturation bits 0–3 flag counts at ≥99.5% of the configured ±320 g range (absolute count ≥30565 at 10.417 mg/LSB), rather than assuming the int16 rails equal the sensor's measurement limit. These flags cover the armed session, so an earlier error can conservatively flag the selected shot. Checks cannot establish externally verified synchronization or every physical sample loss.

For version 1 and 2, estimated sample times come from a linear fit between each sensor's batch-ending sample index and its software FIFO drain timestamp, using a median intercept to reduce latency outliers. The decoder exports the fitted rate and timing residual scatter, trims each sensor to the requested pre/post window using that estimate (or the full start/end span when trigger_us is 0), and also exports the original drain timestamps. The oscillator trim prediction remains separate metadata. A software drain timestamp is not a sample-edge timestamp: independent sensor clocks, startup skew and FIFO latency prevent claiming the document's ≤1-sample synchronization target. Fitting cannot repair missing physical samples; firmware quality flags still make the recording invalid.

For version 2 through 5 automatic triggers, the decoder first maps sensor timing, then shifts every sensor's time axis by the mapped time of the persisted triggering sample. That acquired sample is exactly at `t=0` in the plot and export; the same common shift preserves relative sensor timing. Version 3 and 4 use explicit hardware-derived time axes; version 5 uses original ESP32 interrupt times without clock fitting. This is sample-level threshold detection (nominally about 130 microseconds per sample at 7,680 Hz), not interpolation of the analog crossing between samples or proof of coincident sample edges. The original C6 trigger timestamp is preserved in metadata. Manual and version 1 recordings retain their software-trigger time origin.

Version 4 replaces `IM4D` plus decimated `IM4H` anchors with `IM4S` sample frames. The 19-byte record prefix is followed by a uint32 absolute sensor timestamp for the first sample and 1–256 seven-byte rows: three int16 XYZ counts, then a uint8 timestamp increment. The first increment must be zero; subsequent increments are nonzero. They accumulate exactly, without loss, into original uint32 sensor counters. Intervals larger than 255 ticks start another frame with a full absolute counter. `IM4C` clock calibration and `IM4T` threshold records remain unchanged. Every sample has an original hardware timestamp; no per-sample interpolation or resampling is required. Both NPZ exporters include `imuN_sensor_ticks` alongside `imuN_time_s`. The timestamp update uses the same partition layout and reads older captures.

Version 5 is the current standalone format. `IM4S` absolute counters and increments instead encode ESP32 microseconds since `started_us`, captured at entry to each sensor's INT1 data-ready ISR. XYZ is read directly at 10 MHz SPI; FIFO is bypassed and `IM4H`/`IM4C` records are not accepted. All samples retain their original timing on the common clock. Counters must increase across frames and cannot exceed their append timestamp. Deltas above 255 us begin a new absolute frame. No clock fitting or resampling is applied. NPZ exports include `imuN_drdy_time_us`, explicit `imuN_time_s` and original counts. Version 5 threshold deadlines use the acquired sample's native interrupt timestamp. GPIO interrupt latency and physical pulse mismatch are not certified by these timestamps; see [synchronization.md](synchronization.md).

## Validation

Build with ESP-IDF 6.0.1 and flash both the partition table and application when upgrading from the original partition layout. Ordinary later application updates preserve the recording partition; `erase-flash` destroys recordings. The application partition is now 1 MiB; verify image fit on each build. The timestamp update uses the existing expanded partition layout.

Host integrity tests: `python -m unittest discover -s tests -v`. Device acceptance should cover the 10+3 s window with a late trigger, a no-trigger timeout, automatic triggering, cancellation, refusal to overwrite, download identity after a hard reset and a real power-cycle, and the original USB/TCP/browser streaming modes. Inspect per-sensor counts/rates and actual common-impulse timing, not just the quality mask. Mounting, high-g calibration, sync connector and field-shot acceptance remain physical tests.
