# BowVib: four LSM6DSV320X vibration channels

## On-board shot recording

The firmware now also stores triggered shots in the ESP32-C6's internal flash. The unified phone page defaults to a **50 g** pulse trigger with **10 s before + 3 s after**. An early pulse keeps the available history; with no pulse, acquisition continues until flash capacity and the full session is saved. The 8 MiB flash holds a 1 MiB firmware partition and a **6.9375 MiB recording partition**; a small reserve accommodates the final RAM queue. Duration depends on sensor count, actual rates and frame overhead. A pulse near capacity can have an incomplete post-window, flagged for review. Pre/post settings and threshold can be changed in the page or `shot_tool.py`. Flash is erased before acquisition. A completed shot survives reset and must be explicitly deleted before the next shot. Existing live capture modes remain available.

The expanded layout moves recording storage from `0x210000` to `0x110000`. Download any saved BVR before installing this partition table: existing recordings require migration and are not preserved by simply flashing the new layout. The current firmware fits in 1 MiB; future builds must also pass the partition size check.

New standalone recordings read XYZ at 10 MHz SPI directly on each sensor INT1 data-ready interrupt. Each sample retains its original ESP32 interrupt timestamp; the FIFO is bypassed. All sensor traces retain that common time axis, including real differences in pulse arrival; their sample edges remain independent. Timing diagnostics appear in the phone plot and exported metadata. Older recordings remain readable. See [timing and characterization](docs/synchronization.md).

```powershell
python shot_tool.py --port COM92 arm --pre 10 --post 3
# Automatic 50 g trigger is enabled by default; optionally trigger manually.
python shot_tool.py --port COM92 trigger
python shot_tool.py --port COM92 download shot.bvr
python shot_tool.py export shot.bvr exports
python shot_tool.py analyze shot.bvr shot.png
```

See [recording instructions and format](docs/recording.md), [customer requirement coverage](docs/customer-requirements.md), and [calibration procedure](docs/calibration.md). The partition table changed: rebuild and flash the table/application together for the first upgrade. Four channels at 7,680 Hz are retained; six channels, 10 kHz and verified hardware sync require additional hardware work.

ESP-IDF firmware for an ESP32-C6-MINI-1-H8 and four LSM6DSV320X IMUs. The firmware enables each IMU's high-g accelerometer at its maximum 7,680 samples/s and +/-320 g range. Each IMU writes to its own FIFO; the C6 polls and drains those FIFOs in order 1, 2, 3, 4 over one shared SPI bus. Samples stream live over USB Serial/JTAG, raw TCP, or directly to a phone browser over Wi-Fi.

The H8 module provides 8 MB flash and no PSRAM. Wi-Fi capture uses a 32 KiB FreeRTOS stream buffer and a separate transmit task so brief network stalls do not block FIFO draining. This is a short-term queue, not an on-board recording buffer; sustained Wi-Fi throughput still needs to keep up with the four live channels. Four channels produce 30,720 XYZ samples/s total, or 184,320 payload bytes/s before frame headers. The host utilities decode signed 16-bit counts into `int32` arrays and store them in compressed `.npz` archives for offline analysis.

## Wiring from the supplied schematic

| Signal | C6 GPIO |
| --- | ---: |
| SPI MISO | 2 |
| SPI SCK | 6 |
| SPI MOSI | 7 |
| IMU A-D CS | 16, 17, 18, 19 |
| INT A-D | 20, 21, 22, 23 |
| LED1-LED3 | 3, 4, 5 |
| USB D- / D+ | 12 / 13 |

GPIO16/17 are also UART0 TX/RX. UART console output is disabled so it cannot toggle those chip-select pins or corrupt the USB binary stream. Standalone recording requires sensor INT1 A-D on GPIO20, 21, 22 and 23 respectively, confirmed by the user. Live FIFO streaming still uses polling. IMU A-D correspond to `present_mask` bits 0-3 and are also called IMU 1-4.

## Capture

Use Python 3.10 or later and install the host dependencies:

```powershell
python -m pip install -r requirements.txt
```

Build and flash from an activated ESP-IDF terminal. This project was built and device-tested with ESP-IDF 6.0.1; its CMake dependencies name the SPI, GPIO, and USB Serial/JTAG driver components explicitly. On Windows, activate your installation with its `export.ps1` first.

```powershell
idf.py set-target esp32c6
idf.py build
idf.py -p COM92 flash
```

Find the USB Serial/JTAG port, then capture ten seconds (COM92 is the port used for validation; change it for your PC):

```powershell
python -m serial.tools.list_ports -v
python board_control.py COM92 STATUS
python inspect_raw.py --port COM92 --seconds 10 --save raw_capture.npz
```

## Wi-Fi capture

The ESP32-C6 starts a local WPA2 access point, a browser page, and a TCP capture server:

- SSID: `BowVib-IMU`
- Password: `BowVib320x`
- Phone/PC browser page: `http://192.168.4.1/` (port 80)
- Raw TCP stream for Python receivers: `192.168.4.1:3333`

### Direct phone capture (no PC needed)

1. Power the board. USB can provide power; no USB data connection is needed.
2. Connect the phone to `BowVib-IMU` with password `BowVib320x`. Stay connected if the phone warns that this network has no internet.
3. Open **http://192.168.4.1/** in the phone browser. Use `http`, not `https`, and omit `:3333`.
4. Press **Start recording**. The default 50 g pulse selects up to 10 s before and 3 s after; no pulse records until storage is full and saves the full session.
5. The plot opens around the trigger. Pinch with two fingers or scroll to zoom; drag to pan. **Center event** returns to the pulse, **Full view** shows the recording, and Shift+drag selects a time range. Toggle sensors or choose **XYZ magnitude** to see the saved threshold. Save the raw `.bvr`, `.npz`, or plot PNG. Save and delete the stored recording before recording again.

The ESP32 serves the complete page and records into internal flash. Recording continues if the browser disconnects, and the saved recording survives power-off. The browser validates the downloaded recording and creates its NPZ and zoomable XYZ plot locally. One recording fits on the board at a time; it is never automatically overwritten.

New automatic recordings retain the triggering sensor and sample index. The first observed sample at or above the threshold becomes time zero in plots and exports, removing the late marker caused by FIFO/task processing delay. Each channel keeps its native sampling times on the common ESP32 clock; interrupt latency and physical pulse skew still require characterization. The updated browser and `shot_tool.py` read BVR versions 1 through 5; older recordings retain their original software trigger time. Flash the updated application to enable the new recorder and interface.

The phone page checks manifest/page CRCs, page order, timestamps and per-sensor sample continuity. Quality and saturation warnings are shown beside the plot; flagged raw data remains downloadable for diagnosis. Phone archives retain little-endian signed `int16` channels in an uncompressed NPZ ZIP; Python archives use `int32` channels and compression. Both open in `inspect_raw.py` and preserve full channel counts. The phone archive includes per-sensor estimated time arrays and recording metadata. It omits the redundant common-length `samples_lsb` array; the inspector constructs it on load.

Port 3333 carries the raw command/binary protocol and cannot display an HTTP webpage.

### Python receiver over Wi-Fi

Connect the PC to that Wi-Fi network, then run the receiver over TCP:

```powershell
python inspect_raw.py --wifi --seconds 5 --save wifi_capture.npz --plot-png wifi_capture.png --no-gui
```

The receiver requests `STATUS`, starts acquisition, reads the IMU FIFO stream, stops after the requested duration, and saves the raw counts and metadata to the `.npz` file. Open it later with `python inspect_raw.py wifi_capture.npz`. USB capture remains available with `--port COM92`. The CLI refuses to overwrite existing archives unless `--overwrite` is supplied.

Captures include whichever sensors respond. To rescan the chip-select lines or run the on-board LED sequence, use:

```powershell
python board_control.py COM92 SCAN
python board_control.py COM92 LEDTEST
python board_control.py COM92 CSATEST
python board_control.py COM92 HELP
```

`LEDTEST` flashes GPIO3, GPIO4, and GPIO5 one at a time, then together. The firmware assumes active-high LEDs. A board reset also flashes the sequence once.

`CSATEST` temporarily releases GPIO16 from SPI chip-select control, toggles CS_A high/low once per second for ten transitions, then restores the SPI connection and leaves the pin high. Use a meter or oscilloscope between the ESP32 GPIO16 pad and the IMU A CS pad to check the PCB connection. Capturing and scanning are unavailable during this ten-second test.

Both the CLI and browser downloads contain four full XYZ count arrays (`imu1_samples_lsb` through `imu4_samples_lsb`), per-sensor sample counts and measured rates, active `sensor_ids`, the configured 7,680 Hz rate, 320 g full scale, duration, output bit width, and an overrun/status mask. An absent sensor has an empty `(0, 3)` array. `samples_lsb` is an equal-length view of the active channels for plotting; full channel arrays retain every received sample. `sample_rate_hz` is the mean measured rate of active sensors. Browser downloads also include trim-predicted rates. Use `--counts` to plot LSB counts; otherwise values are shown in g using 10.417 mg/LSB.

For older browser archives that contain only individual IMU arrays, the inspector constructs the common-length view on load. Files without sample-rate metadata default to 7,680 Hz; specify `--rate 80000 --fs 200` when opening legacy 80 kHz / 200 g recordings.

## PC-hosted capture page (optional)

The PC hosts a control page at `http://127.0.0.1:8765/`; the board performs acquisition and transfers the recording after it finishes. Use the direct data-ready firmware from this branch (recording format v5) with its recording partition table. This script does not flash or update the board.

Customer quick start on Windows (Python 3.10 or newer and Git installed):

```powershell
git clone --branch experiment/esp32c6-h8-lsm6dsv320x --single-branch https://github.com/Fajdiga/BowVib.git
cd BowVib
python -m pip install -r requirements.txt
python capture_button.py --wifi
```

Connect the PC to the `BowVib-IMU` Wi-Fi network before running the last command, then open `http://127.0.0.1:8765/`. Repository access is required to clone. These examples control a real board; `capture_button.py` has no simulated-data mode.

If the repository is already downloaded, run commands from its folder. Install the Python dependencies once:

```powershell
python -m pip install -r requirements.txt
```

Run the capture page over USB (replace `COM92` with the board's port from Windows Device Manager, and close any serial monitor using it):

```powershell
python capture_button.py --port COM92
```

Or connect the PC to the `BowVib-IMU` access point and run:

```powershell
python capture_button.py --wifi
```

Keep that terminal open and visit `http://127.0.0.1:8765/` in a browser. Wait for **Board ready**. If the board already holds a recording, the script first downloads and verifies it. To change the local page port or recording folder:

```powershell
python capture_button.py --wifi --web-port 8766 --output-dir "C:\BowVibCaptures"
```

With this example, open `http://127.0.0.1:8766/`. Relative output paths are resolved from the terminal's current folder.

For a typical pulse recording:

1. Set **Trigger** to `50` g, **Pre-buffer** to `10` s, and **Post-buffer** to `3` s.
2. Press **Start trigger capture**. Wait at least ten seconds before the event if you need a full ten-second history.
3. Apply the pulse, or press **Trigger now** to trigger manually. Use `0` g to disable automatic triggering.
4. Wait for capture, download, and validation to finish. The page shows the PC save location and any quality warnings.
5. Use **Plot capture**, **Save plot as PNG**, or **Save capture to this device**. Start another capture when ready; the previous PC files are retained.

Open the printed page address. This script uses the same on-board direct data-ready recorder as the built-in webpage: HTTP `/shot/*` commands over Wi-Fi or `SHOT*` commands over USB. It does not stream sensor FIFO data to the PC. Use firmware supporting direct recording format v5; older FIFO recordings are rejected rather than silently substituted.

**Start trigger capture** sends the threshold and pre/post windows to the board. Automatic pulse detection and capture timing run on the board. Set the threshold to zero for manual triggering, then press **Trigger now**. Limits match the board: up to 10 seconds before the trigger, 3 seconds after, and 0.02 to 320 g for an automatic threshold. An early pulse retains only history available since arming. Without a trigger, the board records until storage fills and saves the full session.

**Start capture** keeps the selected interval (up to 10 seconds) before a manual trigger issued by the PC after that interval, with a zero post-buffer. Keep the PC connected in this mode. Command/polling latency shifts the window's position; it is not an exact acquisition deadline enforced by firmware.

After capture ends, the PC downloads and validates the original recording, then saves timestamped `.bvr` and `.npz` files under `captures/` (override with `--output-dir`). The NPZ uses the same export code as `shot_tool.py`, preserving original sample indices, data-ready timestamps, trigger metadata, and quality warnings. A new capture replaces the board copy only after the previous recording has been downloaded, validated, and written to local files. Existing saved board recordings are recovered automatically when the script connects. Failed downloads leave the board copy intact.

**Save capture to this device** downloads the NPZ to the browser device. **Plot capture** uses the saved sample times, including negative pre-trigger times; **Save plot as PNG** exports the plot. Unequal sensor counts alone do not establish sample loss; review the decoder's timing and quality warnings.

**Cancel capture** uses the board's cancel command. Cancelling before a trigger discards that session, as on the built-in webpage; cancelling during the post-buffer can save an incomplete recording with quality warnings. Ctrl+C cancels a capture started by this script, waits for completion, and downloads any committed recording. It does not cancel a capture started by another client. Use one controlling client at a time. If communication fails, reconnect to recover any saved board recording.

For phone control, keep this script running and open `http://<PC-local-IP>:8765/` on the same network. Allow Python through Windows Firewall on the private network if prompted. Wi-Fi board access now uses HTTP (normally port 80); `--wifi host:port` supports a different HTTP port. The old raw-stream `--wifi-port` option is no longer used.

If the page reports a connection error, check the selected USB port or reconnect the PC to the board's Wi-Fi. Status polling retries automatically. A validation failure preserves the raw PC file and board copy, blocks replacement, and requires restarting the script to retry. A format-v5 or storage-unavailable error requires checking the installed firmware/partition table. Do not delete the board copy to work around a failed download. Press Ctrl+C once to exit and let cleanup finish.

For a plot image that does not need an interactive GUI, use `--plot-png`:

```powershell
python inspect_raw.py raw_capture.npz --plot-png raw_capture.png --no-gui
```

Open a saved capture interactively with:

```powershell
python inspect_raw.py raw_capture.npz
```

Use `--demo` to preview the cursor interface without hardware.

### Status LEDs

The active-high board LEDs on GPIO3-5 briefly sweep at boot, then report status:

| LED | Meaning | Indication |
| --- | --- | --- |
| LED1 / GPIO3 | Firmware alive | Steady on after startup. |
| LED2 / GPIO4 | IMU discovery | Solid = all four found; slow blink = 1-3 found; fast blink = none found. |
| LED3 / GPIO5 | Wi-Fi / capture | Off = AP failed; short pulse every 2 s = waiting for a capture client; solid = client connected and idle; fast blink = Wi-Fi capture; 1 Hz blink = USB capture; double flash every 3 s = overrun reported. |

Use `python board_control.py COM92 LEDTEST` to flash each LED individually. The access point accepts two Wi-Fi stations; either a phone browser or a PC receiver can own a capture.

## Project files

| File | Purpose |
| --- | --- |
| `main/lsm6dsv320x.c` | SPI discovery, IMU configuration, FIFO reads, and CS_A diagnostics. |
| `main/capture.c` | FIFO polling, framing, counters, stop/tail handling, and Wi-Fi transmit queue. |
| `main/usb.c`, `main/wifi_stream.c` | Command handlers and USB/TCP transports. |
| `main/web_capture.c`, `main/web/` | Board-hosted phone page, WebSocket capture, plotting, and local NPZ downloads. |
| `main/led_status.c` | Startup sequence and status indications. |
| `inspect_raw.py` | Shared frame validation, archive handling, CLI capture, and offline plots. |
| `capture_button.py` | Local HTTP page controlling on-board captures and downloading completed recordings. |
| `board_control.py` | USB status and diagnostic commands. |

## Stream protocol

All integers are little-endian. Sensor IDs in binary frames are zero-based (0-3); archive IDs are one-based (1-4).

| Frame | Layout |
| --- | --- |
| Data | `IM4D`, sensor (`uint8`), row count (`uint16`, 1-256), then signed XYZ (`int16` each, 6 bytes per row). |
| End | `IM4E`, four sample counts (`uint32` each), duration in ms (`uint32`), status mask (`uint8`). |

Status bits 0-3 indicate FIFO overrun or a FIFO read failure for IMUs A-D. Bit 7 indicates output transport failure or Wi-Fi queue overflow. A nonzero mask means the recording needs review. The firmware stays busy through footer transmission and stops a Wi-Fi send that stalls for one second.

## Device validation

For a device smoke test, close other port readers, run `STATUS` and `SCAN`, then record and reopen an archive:

```powershell
python board_control.py COM92 STATUS
python board_control.py COM92 SCAN
python inspect_raw.py --port COM92 --seconds 10 --save usb_test.npz --plot-png usb_test.png --no-gui
python inspect_raw.py usb_test.npz --no-gui
```

Check that all expected sensors appear, frame counts match the footer, the status mask is zero, and each measured rate is close to its trim-predicted rate. For standalone Wi-Fi validation, close USB receivers, connect directly to `BowVib-IMU`, open `http://192.168.4.1/`, capture twice, and save both archives. Reopen them with `inspect_raw.py` to check repeated captures and storage. The optional PC-hosted page can be checked the same way with `capture_button.py`.
