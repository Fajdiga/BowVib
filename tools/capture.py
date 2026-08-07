#!/usr/bin/env python3
"""
BowVib capture tool — talks to the ESP32-S3 over the native USB port
(USB-Serial-JTAG). Sends START, waits for the buffer to fill, sends DUMP,
parses the framed binary (10-byte FIFO rows, 20-bit signed XYZ) and saves
to .npy / .csv, optionally plotting.

Usage:
    python capture.py --port COM5 --odr 20000 --fs 200 --out run1.npy
    python capture.py --port COM5 --stream --seconds 5 --out run1.npy

FIFO row layout (little-endian):
    byte0      tag (high nibble 0x1 == accelerometer)
    bytes 1..3 X (24-bit container, 20-bit signed)
    bytes 4..6 Y
    bytes 7..9 Z
"""
import argparse
import struct
import sys
import time

import serial

MGLSB = {50: 0.095, 100: 0.191, 200: 0.381}


def read_line(ser, timeout=60.0):
    """Read one CR/LF-terminated line (text)."""
    ser.timeout = timeout
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("timeout waiting for line: " + buf.decode(errors="replace"))
        if b in (b"\r", b"\n"):
            if buf:
                return buf.decode(errors="replace").strip()
        else:
            buf += b


def read_exact(ser, n, timeout=60.0):
    """Read exactly n bytes."""
    ser.timeout = timeout
    chunks = bytearray()
    while len(chunks) < n:
        d = ser.read(n - len(chunks))
        if not d:
            raise TimeoutError(f"timeout: got {len(chunks)}/{n} bytes")
        chunks += d
    return bytes(chunks)


def wait_for(ser, prefix, timeout=120.0):
    """Read lines until one startswith prefix; return that line."""
    end = time.time() + timeout
    while time.time() < end:
        line = read_line(ser, timeout=max(1.0, end - time.time()))
        print("  dev>", line)
        if line.startswith(prefix):
            return line
    raise TimeoutError(f"no '{prefix}' line within {timeout}s")


def kv(line, key):
    for tok in line.split():
        if tok.startswith(key):
            return tok.split("=", 1)[1]
    return None


def parse_rows(data, mglsb):
    import numpy as np
    rows = np.frombuffer(data, dtype=np.uint8).reshape(-1, 10)
    def axis(c0, c1, c2):
        v = rows[:, c0].astype(np.int32) | (rows[:, c1].astype(np.int32) << 8) \
            | (rows[:, c2].astype(np.int32) << 16)
        v = np.where(v & 0x80000, v - 0x100000, v)   # sign-extend 20-bit
        return v
    xyz_lsb = np.stack([axis(1, 2, 3), axis(4, 5, 6), axis(7, 8, 9)], axis=1)
    return xyz_lsb, xyz_lsb * (mglsb / 1000.0)   # LSB, g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="USB-Serial-JTAG COM port")
    ap.add_argument("--odr", type=int, default=20000, choices=[20000, 40000, 80000])
    ap.add_argument("--fs", type=int, default=200, choices=[50, 100, 200])
    ap.add_argument("--out", default="bowvib.npy", help="output .npy (and .csv)")
    ap.add_argument("--stream", action="store_true", help="live stream instead of store")
    ap.add_argument("--seconds", type=float, default=5.0, help="stream duration")
    ap.add_argument("--plot", action="store_true", help="plot after capture")
    args = ap.parse_args()

    ser = serial.Serial(args.port, baudrate=115200, timeout=2.0)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # flush banner
    while ser.in_waiting:
        ser.readline()

    def cmd(c):
        ser.write((c + "\n").encode())
        time.sleep(0.05)

    cmd(f"SET ODR {args.odr}")
    cmd(f"SET FS {args.fs}")

    if args.stream:
        print(f"streaming ~{args.seconds}s @ {args.odr} Hz, ±{args.fs}g ...")
        cmd("STREAM")
        read_line(ser, timeout=2.0)   # consume "STREAM START ..." preamble (text before binary)
        raw = bytearray()
        end = time.time() + args.seconds
        while time.time() < end:
            d = ser.read(8192)
            if d:
                raw += d
        cmd("STOP")
        # consume any trailing text line(s)
        time.sleep(0.2)
        while ser.in_waiting:
            print("  dev>", ser.readline().decode(errors="replace").strip())
        xyz_lsb, xyz_g = parse_rows(bytes(raw[:len(raw) // 10 * 10]), MGLSB[args.fs])
    else:
        print(f"capturing to PSRAM @ {args.odr} Hz, ±{args.fs}g ...")
        cmd("START")
        line = wait_for(ser, "CAPTURE ")          # "CAPTURE DONE ..." or "CAPTURE STOPPED ..."
        overrun = kv(line, "overrun=")
        if overrun == "1":
            print("WARNING: FIFO overran — samples were lost. Raise SPI clock or lower ODR.")
        cmd("DUMP")
        hdr = read_exact(ser, 32)
        if hdr[:4] != b"BOWV":
            sys.exit(f"bad frame magic: {hdr[:4]!r}")
        ver, odr_hz, fs_g, row_bytes, n, ovr = struct.unpack("<H I H H I H", hdr[4:20])
        print(f"frame: v{ver} odr={odr_hz}Hz fs=±{fs_g}g rows={n} row_bytes={row_bytes} overrun={ovr}")
        data = read_exact(ser, n * row_bytes)
        xyz_lsb, xyz_g = parse_rows(data, MGLSB[fs_g])

    import numpy as np
    np.save(args.out, xyz_g)
    np.save(args.out.replace(".npy", "_lsb.npy"), xyz_lsb)
    csv = args.out.replace(".npy", ".csv")
    np.savetxt(csv, xyz_g, delimiter=",", header="x_g,y_g,z_g")
    t = np.arange(len(xyz_g)) / args.odr
    print(f"saved {len(xyz_g)} samples -> {args.out}, {csv}")
    print(f"duration {t[-1]:.2f}s   mean(g) x={xyz_g[:,0].mean():.3f} "
          f"y={xyz_g[:,1].mean():.3f} z={xyz_g[:,2].mean():.3f}  "
          f"std(g) {xyz_g.std(0)}")

    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(10, 6))
        ax[0].plot(t, xyz_g)
        ax[0].set(xlabel="time [s]", ylabel="g", title="BowVib time domain")
        ax[0].legend(["x", "y", "z"])
        # magnitude spectrum of z
        nfft = 1 << int(np.log2(min(len(xyz_g), 1 << 16)))
        z = xyz_g[:, 2] - xyz_g[:, 2].mean()
        Z = np.abs(np.fft.rfft(z * np.hanning(len(z))[:nfft]))[: nfft // 2]
        f = np.arange(len(Z)) * (args.odr / nfft)
        ax[1].semilogy(f, Z)
        ax[1].set(xlabel="freq [Hz]", ylabel="|Z|", title="Z spectrum (DC removed)")
        fig.tight_layout()
        plt.show()

    ser.close()


if __name__ == "__main__":
    main()
