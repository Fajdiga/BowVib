# BowVib: four LSM6DSV320X vibration channels

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

GPIO16/17 are also UART0 TX/RX. UART console output is disabled so it cannot toggle those chip-select pins or corrupt the USB binary stream. FIFO draining uses polling; the interrupt pins are not required for acquisition. IMU A-D correspond to `present_mask` bits 0-3 and are also called IMU 1-4.

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
4. Press **Start 10-second capture**. Keep the page visible and the screen awake until it finishes.
5. Press **Save capture to this device** to download the `.npz` file. **Plot capture** shows XYZ; **Save plot as PNG** saves the image.

The ESP32 serves the complete page and streams binary samples over a WebSocket. It stops acquisition automatically after ten seconds, even if the browser timer is delayed. The phone holds the recording in memory and creates the archive locally. Save it before refreshing the page, closing the tab, or starting another capture. Only one browser can hold the capture connection at a time; close other capture tabs if the page keeps reconnecting. A disconnected capture is stopped and discarded.

The phone page validates frame/footer counts and refuses to save captures with an overrun/transport error. Phone archives retain little-endian signed `int16` channels in an uncompressed NPZ ZIP; Python archives use `int32` channels and compression. Both open in `inspect_raw.py` and preserve full channel counts. The phone archive omits the redundant common-length `samples_lsb` array; the inspector constructs it on load.

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

Run the capture page over USB:

```powershell
python capture_button.py --port COM92
```

Or connect the PC to the `BowVib-IMU` access point and run:

```powershell
python capture_button.py --wifi
```

Open the printed page address. The board remains idle until **Start 10-second capture** is pressed. It then samples the detected IMUs, drains their FIFOs in order 1-2-3-4, stops after ten seconds, and reports per-sensor sample counts and rates. The page supports repeated captures. **Save capture to this device** downloads independent raw XYZ arrays with a unique timestamped `.npz` filename. **Plot capture** displays the three axes for each IMU in the browser; **Save plot as PNG** downloads that plot.

Each device can run at a slightly different actual ODR. The page and NPZ therefore show both the measured rate (samples divided by capture duration) and the trim-predicted rate from each IMU's `INTERNAL_FREQ_FINE` value. Unequal counts alone do not prove loss. A clear FIFO/stream-overrun flag reports detected overflow, but a clear flag cannot detect every possible missing or malformed sample; compare measured and trim-predicted rates and verify frame/footer counts when validating a capture.

Malformed frames, absent-sensor data, and frame/footer count mismatches are rejected. Invalid browser captures cannot be downloaded as successful recordings. If a USB stream fails, restart the capture page to reconnect; Wi-Fi reconnects on the next capture attempt. Use one host receiver per board. USB and TCP commands share a firmware lock, and commands other than `STOP` are ignored on the active binary stream to keep text replies out of sample data. A client on the other transport can request `STATUS`; its capture/diagnostic commands return `ERR busy`.

For phone control/storage, keep the PC and capture page running and open the printed PC address in the phone browser while both are on the same local network. Or connect both the PC and phone to `BowVib-IMU` when using Wi-Fi capture; the ESP32 access point accepts two stations. The phone saves the downloaded NPZ file to Downloads. If Windows Firewall prompts, allow Python on the private network. This button-triggered clip is ten seconds after the button press; the final pre-event/post-event trigger window can be added after capture quality is established.

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
| `capture_button.py` | Local HTTP page for repeated ten-second captures and downloads. |
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
