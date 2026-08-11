# BowVib IMU firmware and raw-data inspector

This repository contains the ESP-IDF firmware for the ESP32-S3 and
IIS3DWB10IS accelerometer, plus one host utility: `inspect_raw.py`.

Install the host dependencies:

```powershell
python -m pip install numpy matplotlib pyserial
```

Record 10 seconds of raw 20-bit XYZ samples and open the inspector:

```powershell
python inspect_raw.py --port COM53
```

The board records into ESP32 PSRAM first, with no sample data travelling over
USB during the 10-second measurement. After acquisition stops, the complete
recording is dumped over USB and stored in `raw_capture.npz` as exact signed
LSB counts together with its sample rate, full-scale, bit depth, and overrun
status. No CSV copy is produced. To open it again:

```powershell
python inspect_raw.py raw_capture.npz
```

The capture default is 40,000 samples/s. The PCB's SPI link is reliable at
8 MHz; 10 MHz and above corrupted FIFO reads during hardware testing. Because
each 20-bit FIFO sample occupies 10 bytes, 80,000 samples/s leaves no practical
bus margin at 8 MHz and is not suitable for lossless recording on this board.
The saved `sample_rate_hz` is measured from the returned row count and the
ESP32 capture duration, so cursor timing uses the observed rate rather than
only the nominal ODR setting.

Use `--seconds` to request another duration. Before starting, the script checks
the board's PSRAM capture capacity and refuses a duration that cannot fit:

```powershell
python inspect_raw.py --port COM53 --seconds 5
```

Integer `.npy` files are treated as raw LSB counts. Floating-point `.npy`
files are treated as acceleration in g. Supply the correct rate when the file
does not contain metadata:

```powershell
python inspect_raw.py measurement.npy --rate 80000 --fs 50
```

The red and purple vertical lines are cursors A and B. Drag either line, then
click **Zoom A-B** or press **Z**. The readout gives both sample times, XYZ
amplitudes, delta-time, `1 / delta-time`, and amplitude differences. Place the
cursors on equivalent points of adjacent cycles to read the signal frequency.
The mouse wheel zooms around the pointer; **R** restores the full recording.

Try the interface without hardware:

```powershell
python inspect_raw.py --demo
```

Build and flash the firmware with the normal ESP-IDF commands:

```powershell
idf.py build
idf.py flash
```

`partitions.csv` is intentionally retained: `sdkconfig.defaults` selects it as
the firmware's required custom ESP-IDF partition table.
