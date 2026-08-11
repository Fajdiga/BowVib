#!/usr/bin/env python3
"""Capture and interactively inspect raw BowVib IMU samples.

Examples:
    python inspect_raw.py --port COM53
    python inspect_raw.py raw_capture.npz
    python inspect_raw.py old_counts.npy --rate 80000 --fs 50
    python inspect_raw.py --demo

The plot has two draggable cursors. Their readout shows time, XYZ amplitude,
delta-time, amplitude differences, and 1/delta-time. Use the "Zoom A-B"
button (or press Z) to zoom to the interval between the cursors.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROW_BYTES = 10
ACCEL_TAG = 0x10
MILLIG_PER_LSB = {50: 0.095, 100: 0.191, 200: 0.381}
ODR_CHOICES = (2500, 5000, 10000, 20000, 40000, 80000)


@dataclass
class DataSet:
    samples: np.ndarray
    sample_rate_hz: float
    unit: str
    source: str


@dataclass
class CaptureResult:
    samples_lsb: np.ndarray
    sample_rate_hz: float
    configured_sample_rate_hz: int
    capture_duration_s: float
    full_scale_g: int
    output_bits: int
    overrun: bool


def normalize_xyz(samples: np.ndarray) -> np.ndarray:
    """Return a finite Nx3 array without changing its numeric values."""
    values = np.asarray(samples)
    if values.ndim != 2:
        raise ValueError(f"expected a 2-D sample array, got shape {values.shape}")
    if values.shape[1] != 3 and values.shape[0] == 3:
        values = values.T
    if values.shape[1] != 3:
        raise ValueError(f"expected Nx3 XYZ samples, got shape {values.shape}")
    if len(values) < 2:
        raise ValueError("at least two samples are required")
    if not np.isfinite(values).all():
        raise ValueError("sample data contains NaN or infinite values")
    return values


def decode_rows(data: bytes | bytearray | memoryview) -> np.ndarray:
    """Decode tagged 10-byte FIFO rows into signed 20-bit XYZ counts."""
    raw = np.frombuffer(data, dtype=np.uint8)
    if raw.size % ROW_BYTES:
        raise ValueError("raw dump length is not a whole number of rows")
    rows = raw.reshape(-1, ROW_BYTES)
    invalid = np.flatnonzero(rows[:, 0] != ACCEL_TAG)
    if invalid.size:
        index = int(invalid[0])
        tags, counts = np.unique(rows[invalid, 0], return_counts=True)
        summary = ", ".join(
            f"0x{int(tag):02x}:{int(count):,}"
            for tag, count in zip(tags[:6], counts[:6], strict=True)
        )
        raise ValueError(
            f"invalid FIFO tag 0x{rows[index, 0]:02x} at row {index}; "
            f"{invalid.size:,}/{len(rows):,} rows invalid ({summary})"
        )

    xyz = np.empty((len(rows), 3), dtype=np.int32)
    for axis, offset in enumerate((1, 4, 7)):
        value = (
            rows[:, offset].astype(np.int32)
            | (rows[:, offset + 1].astype(np.int32) << 8)
            | (rows[:, offset + 2].astype(np.int32) << 16)
        )
        value &= 0xFFFFF
        xyz[:, axis] = np.where(value & 0x80000, value - 0x100000, value)
    return xyz


def read_line(serial_port, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    buffer = bytearray()
    while time.monotonic() < deadline:
        byte = serial_port.read(1)
        if not byte:
            continue
        if byte in (b"\r", b"\n"):
            if buffer:
                return buffer.decode("ascii", errors="replace").strip()
        else:
            buffer.extend(byte)
    partial = buffer.decode("ascii", errors="replace")
    raise TimeoutError(f"timed out waiting for board response: {partial}")


def read_exact(serial_port, byte_count: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    result = bytearray()
    while len(result) < byte_count and time.monotonic() < deadline:
        chunk = serial_port.read(min(65536, byte_count - len(result)))
        if chunk:
            result.extend(chunk)
    if len(result) != byte_count:
        raise TimeoutError(
            f"USB dump stopped after {len(result):,}/{byte_count:,} bytes"
        )
    return bytes(result)


def send_and_wait(
    serial_port, command: str, expected: str, timeout: float = 4.0
) -> str:
    serial_port.write((command + "\n").encode("ascii"))
    serial_port.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = read_line(serial_port, max(0.1, deadline - time.monotonic()))
        print(f"  board> {line}")
        if line.startswith("ERR "):
            raise RuntimeError(line)
        if line.startswith(expected):
            return line
    raise TimeoutError(f"board did not reply with {expected!r}")


def line_integer(line: str, key: str) -> int:
    prefix = key + "="
    for token in line.split():
        if token.startswith(prefix):
            return int(token[len(prefix):])
    raise ValueError(f"board response is missing {key}=...: {line}")


def capture_serial(
    port: str, seconds: float, rate: int, full_scale: int
) -> CaptureResult:
    """Store samples in ESP32 PSRAM, then download one framed binary dump."""
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(
            "serial capture needs pyserial: python -m pip install pyserial"
        ) from exc

    serial_port = serial.Serial(port=None, baudrate=115200, timeout=0.05)
    serial_port.dtr = False
    serial_port.rts = False
    serial_port.port = port

    try:
        serial_port.open()
        time.sleep(0.5)
        serial_port.reset_input_buffer()

        # Set 20-bit mode first: firmware rejects an ODR above 10 kHz while
        # left in 16-bit mode by an earlier session.
        send_and_wait(serial_port, "SET BITS 20", "OK BITS=")
        send_and_wait(serial_port, f"SET ODR {rate}", "OK ODR=")
        send_and_wait(serial_port, f"SET FS {full_scale}", "OK FS=")
        status = send_and_wait(serial_port, "STATUS", "STATUS running=")
        capacity_bytes = line_integer(status, "cap_bytes")
        required_bytes = int(np.ceil(seconds * rate)) * ROW_BYTES
        if capacity_bytes < required_bytes:
            maximum_seconds = capacity_bytes / (rate * ROW_BYTES)
            raise RuntimeError(
                f"board PSRAM holds only {maximum_seconds:.3f} s at {rate} samples/s; "
                f"{seconds:g} s needs {required_bytes:,} bytes but capacity is "
                f"{capacity_bytes:,} bytes"
            )

        send_and_wait(serial_port, "START", "STORE START")
        print(
            f"Recording {seconds:g} s into ESP32 PSRAM at "
            f"{rate} samples/s (USB data transfer is idle) ..."
        )
        capture_started = time.monotonic()
        deadline = capture_started + seconds
        next_progress = capture_started + 1.0
        interrupted = False
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_progress:
                    elapsed = min(seconds, now - capture_started)
                    print(f"  recorded {elapsed:.0f}/{seconds:g} s")
                    next_progress += 1.0
                time.sleep(min(0.05, max(0.0, deadline - now)))
        except KeyboardInterrupt:
            interrupted = True
            print("\nCapture stopped by user.")
        capture_status = send_and_wait(serial_port, "STOP", "CAPTURE ", timeout=10.0)
        duration_ms = line_integer(capture_status, "dur_ms")

        serial_port.write(b"DUMP\n")
        serial_port.flush()
        header = read_exact(serial_port, 32, timeout=5.0)
        if header[:4] != b"BOWV":
            raise RuntimeError(f"invalid dump magic {header[:4]!r}")

        version = struct.unpack_from("<H", header, 4)[0]
        configured_rate = struct.unpack_from("<I", header, 6)[0]
        actual_fs = struct.unpack_from("<H", header, 10)[0]
        row_bytes = struct.unpack_from("<H", header, 12)[0]
        sample_count = struct.unpack_from("<I", header, 14)[0]
        overrun = bool(struct.unpack_from("<H", header, 18)[0])
        output_bits = struct.unpack_from("<H", header, 20)[0]
        if version != 1:
            raise RuntimeError(f"unsupported dump version {version}")
        if row_bytes != ROW_BYTES:
            raise RuntimeError(f"unsupported dump row size {row_bytes}")
        if output_bits != 20:
            raise RuntimeError(f"expected 20-bit samples, board reported {output_bits}")
        if configured_rate not in ODR_CHOICES or actual_fs not in MILLIG_PER_LSB:
            raise RuntimeError(
                f"invalid dump metadata: rate={configured_rate}, full-scale={actual_fs}"
            )

        payload_bytes = sample_count * row_bytes
        print(
            f"Recording stopped ({capture_status}). Dumping "
            f"{sample_count:,} samples/{payload_bytes:,} bytes over USB ..."
        )
        payload = read_exact(serial_port, payload_bytes, timeout=120.0)
        samples = decode_rows(payload)

        if not interrupted:
            if duration_ms < seconds * 1000.0 * 0.98:
                raise RuntimeError(
                    f"capture duration is short: board recorded {duration_ms / 1000.0:.6f} s, "
                    f"requested {seconds:g} s"
                )
    finally:
        if serial_port.is_open:
            serial_port.close()

    if overrun:
        print("WARNING: the IMU FIFO overran; this recording contains lost samples.")
    capture_duration_s = duration_ms / 1000.0
    measured_rate = sample_count / capture_duration_s
    if not interrupted and measured_rate < configured_rate * 0.95:
        raise RuntimeError(
            f"effective acquisition rate is only {measured_rate:,.1f} samples/s "
            f"({measured_rate / configured_rate:.1%} of configured "
            f"{configured_rate:,}); check that the updated SPI firmware is flashed"
        )
    print(
        f"Downloaded {len(samples):,} samples "
        f"({capture_duration_s:.6f} s, effective rate {measured_rate:,.3f} samples/s)."
    )
    return CaptureResult(
        samples,
        measured_rate,
        configured_rate,
        capture_duration_s,
        actual_fs,
        output_bits,
        overrun,
    )


def scalar(archive, key: str, default=None):
    if key not in archive.files:
        return default
    return np.asarray(archive[key]).reshape(()).item()


def load_file(
    path: Path,
    rate_override: float | None,
    full_scale: int,
    show_counts: bool,
    unit_override: str | None,
) -> DataSet:
    if not path.is_file():
        raise FileNotFoundError(path)

    metadata_rate = None
    metadata_fs = None
    metadata_unit = None
    is_counts = False

    if path.suffix.lower() == ".npz":
        with np.load(path, allow_pickle=False) as archive:
            if "samples_lsb" in archive.files:
                samples = archive["samples_lsb"]
                is_counts = True
            elif "samples" in archive.files:
                samples = archive["samples"]
            elif "xyz" in archive.files:
                samples = archive["xyz"]
            elif "arr_0" in archive.files:
                samples = archive["arr_0"]
            else:
                raise ValueError(
                    f"{path} has no samples_lsb, samples, xyz, or arr_0 array"
                )
            metadata_rate = scalar(archive, "sample_rate_hz")
            metadata_fs = scalar(archive, "full_scale_g")
            metadata_unit = scalar(archive, "unit")
    elif path.suffix.lower() == ".npy":
        samples = np.load(path, allow_pickle=False)
        is_counts = np.issubdtype(samples.dtype, np.integer)
    else:
        raise ValueError("input must be a .npy or .npz file")

    samples = normalize_xyz(samples)
    rate = float(rate_override if rate_override is not None else metadata_rate or 80000)
    if rate <= 0:
        raise ValueError("sample rate must be greater than zero")
    if rate_override is None and metadata_rate is None:
        print("No sample-rate metadata found; using the firmware default of 80000 Hz.")

    effective_fs = int(metadata_fs if metadata_fs is not None else full_scale)
    if effective_fs not in MILLIG_PER_LSB:
        raise ValueError(f"unsupported full-scale value: {effective_fs}")

    if is_counts and not show_counts:
        samples = samples.astype(np.float64) * (MILLIG_PER_LSB[effective_fs] / 1000.0)
        unit = "g"
    else:
        unit = unit_override or ("LSB" if is_counts else str(metadata_unit or "g"))
    return DataSet(samples, rate, unit, str(path))


def demo_data(show_counts: bool) -> DataSet:
    rate = 80000.0
    time_axis = np.arange(int(rate * 0.025), dtype=np.float64) / rate
    counts = np.column_stack(
        (
            6000 * np.sin(2 * np.pi * 1200 * time_axis),
            3500 * np.sin(2 * np.pi * 2400 * time_axis + 0.4),
            5000 * np.sin(2 * np.pi * 600 * time_axis + 1.0),
        )
    ).round().astype(np.int32)
    if show_counts:
        return DataSet(counts, rate, "LSB", "synthetic demo")
    values_g = counts.astype(np.float64) * (MILLIG_PER_LSB[50] / 1000.0)
    return DataSet(values_g, rate, "g", "synthetic demo")


class InteractiveInspector:
    MAX_VISIBLE_POINTS = 6000

    def __init__(self, data: DataSet) -> None:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.widgets import Button
        except ImportError as exc:
            raise RuntimeError(
                "interactive inspection needs Matplotlib: "
                "python -m pip install matplotlib"
            ) from exc

        self.plt = plt
        self.data = data
        self.values = normalize_xyz(data.samples)
        self.rate = data.sample_rate_hz
        self.duration = (len(self.values) - 1) / self.rate
        self.cursor_times = [self.duration / 3.0, self.duration * 2.0 / 3.0]
        self.dragging: int | None = None
        self._updating_view = False

        self.figure, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
        self.axes = list(axes)
        self.figure.canvas.manager.set_window_title("BowVib raw-data inspector")
        self.figure.subplots_adjust(top=0.88, bottom=0.23, hspace=0.08)
        self.figure.suptitle(
            f"{data.source} | {len(self.values):,} samples | "
            f"{self.rate:g} samples/s"
        )

        axis_names = ("X", "Y", "Z")
        self.data_lines = []
        self.cursor_lines = [[], []]
        self.cursor_markers = [[], []]
        cursor_colors = ("tab:red", "tab:purple")
        for axis_index, axis in enumerate(self.axes):
            line, = axis.plot([], [], color=f"C{axis_index}", linewidth=0.8)
            self.data_lines.append(line)
            axis.set_ylabel(f"{axis_names[axis_index]} [{data.unit}]")
            axis.grid(True, alpha=0.25)
            for cursor_index, color in enumerate(cursor_colors):
                cursor_line = axis.axvline(
                    self.cursor_times[cursor_index], color=color, linewidth=1.2
                )
                marker, = axis.plot([], [], "o", color=color, markersize=5)
                self.cursor_lines[cursor_index].append(cursor_line)
                self.cursor_markers[cursor_index].append(marker)
        self.axes[-1].set_xlabel("time [s]")

        self.readout = self.figure.text(
            0.015, 0.025, "", family="monospace", fontsize=9, va="bottom"
        )
        zoom_axis = self.figure.add_axes((0.73, 0.105, 0.12, 0.055))
        reset_axis = self.figure.add_axes((0.86, 0.105, 0.12, 0.055))
        self.zoom_button = Button(zoom_axis, "Zoom A-B")
        self.reset_button = Button(reset_axis, "Reset")
        self.zoom_button.on_clicked(self.zoom_between_cursors)
        self.reset_button.on_clicked(self.reset_view)

        self.axes[0].set_xlim(0.0, self.duration)
        self.axes[0].callbacks.connect("xlim_changed", self._view_changed)
        self.figure.canvas.mpl_connect("button_press_event", self._mouse_press)
        self.figure.canvas.mpl_connect("motion_notify_event", self._mouse_move)
        self.figure.canvas.mpl_connect("button_release_event", self._mouse_release)
        self.figure.canvas.mpl_connect("scroll_event", self._scroll_zoom)
        self.figure.canvas.mpl_connect("key_press_event", self._key_press)
        self._refresh_view()
        self._refresh_cursors()

    def _sample_index(self, time_seconds: float) -> int:
        return int(np.clip(round(time_seconds * self.rate), 0, len(self.values) - 1))

    def _view_changed(self, _axis=None) -> None:
        if not self._updating_view:
            self._refresh_view()

    def _refresh_view(self) -> None:
        self._updating_view = True
        try:
            left, right = sorted(self.axes[0].get_xlim())
            left = max(0.0, left)
            right = min(self.duration, right)
            first = max(0, int(np.floor(left * self.rate)))
            last = min(len(self.values), int(np.ceil(right * self.rate)) + 1)
            if last <= first:
                last = min(len(self.values), first + 1)

            count = last - first
            if count > self.MAX_VISIBLE_POINTS:
                indices = np.linspace(
                    first, last - 1, self.MAX_VISIBLE_POINTS, dtype=np.int64
                )
                indices = np.unique(indices)
            else:
                indices = np.arange(first, last, dtype=np.int64)
            visible_time = indices.astype(np.float64) / self.rate

            segment = self.values[first:last]
            for axis_index, axis in enumerate(self.axes):
                self.data_lines[axis_index].set_data(
                    visible_time, self.values[indices, axis_index]
                )
                low = float(np.min(segment[:, axis_index]))
                high = float(np.max(segment[:, axis_index]))
                span = high - low
                padding = max(span * 0.08, abs(low) * 0.01, 1e-9)
                axis.set_ylim(low - padding, high + padding)
            self.figure.canvas.draw_idle()
        finally:
            self._updating_view = False

    def _refresh_cursors(self) -> None:
        indices = [self._sample_index(value) for value in self.cursor_times]
        snapped_times = [index / self.rate for index in indices]
        self.cursor_times[:] = snapped_times

        for cursor_index, sample_index in enumerate(indices):
            cursor_time = snapped_times[cursor_index]
            for axis_index in range(3):
                self.cursor_lines[cursor_index][axis_index].set_xdata(
                    [cursor_time, cursor_time]
                )
                self.cursor_markers[cursor_index][axis_index].set_data(
                    [cursor_time], [self.values[sample_index, axis_index]]
                )

        sample_a = self.values[indices[0]].astype(np.float64)
        sample_b = self.values[indices[1]].astype(np.float64)
        delta_t = abs(snapped_times[1] - snapped_times[0])
        frequency = np.inf if delta_t == 0.0 else 1.0 / delta_t
        delta_amplitude = sample_b - sample_a
        half_span = np.abs(delta_amplitude) / 2.0

        def xyz(values: np.ndarray) -> str:
            return "  ".join(
                f"{name}={value:+.7g}" for name, value in zip("XYZ", values)
            )

        frequency_text = "infinite" if not np.isfinite(frequency) else f"{frequency:.7g} Hz"
        self.readout.set_text(
            f"A: t={snapped_times[0]:.9f} s  sample={indices[0]:,}  {xyz(sample_a)} {self.data.unit}\n"
            f"B: t={snapped_times[1]:.9f} s  sample={indices[1]:,}  {xyz(sample_b)} {self.data.unit}\n"
            f"delta-t={delta_t:.9g} s   1/delta-t={frequency_text}   "
            f"delta: {xyz(delta_amplitude)} {self.data.unit}\n"
            f"half |delta| (peak estimate): {xyz(half_span)} {self.data.unit}   "
            "Drag either cursor; Z=zoom A-B; R=reset; wheel=zoom"
        )
        self.figure.canvas.draw_idle()

    def _toolbar_is_active(self) -> bool:
        toolbar = getattr(self.figure.canvas, "toolbar", None)
        return bool(toolbar is not None and toolbar.mode)

    def _mouse_press(self, event) -> None:
        if event.button != 1 or event.inaxes not in self.axes or event.xdata is None:
            return
        if self._toolbar_is_active():
            return
        pixel_x = event.inaxes.transData.transform((event.xdata, 0))[0]
        cursor_pixels = [
            event.inaxes.transData.transform((value, 0))[0]
            for value in self.cursor_times
        ]
        self.dragging = int(np.argmin(np.abs(np.asarray(cursor_pixels) - pixel_x)))
        self.cursor_times[self.dragging] = float(np.clip(event.xdata, 0, self.duration))
        self._refresh_cursors()

    def _mouse_move(self, event) -> None:
        if self.dragging is None or event.inaxes not in self.axes or event.xdata is None:
            return
        self.cursor_times[self.dragging] = float(np.clip(event.xdata, 0, self.duration))
        self._refresh_cursors()

    def _mouse_release(self, _event) -> None:
        self.dragging = None

    def _scroll_zoom(self, event) -> None:
        if event.inaxes not in self.axes or event.xdata is None:
            return
        left, right = self.axes[0].get_xlim()
        factor = 0.65 if event.button == "up" else 1.0 / 0.65
        new_span = (right - left) * factor
        if new_span >= self.duration:
            self.reset_view()
            return
        relative = (event.xdata - left) / (right - left)
        new_left = event.xdata - new_span * relative
        new_left = float(np.clip(new_left, 0.0, self.duration - new_span))
        self.axes[0].set_xlim(new_left, new_left + new_span)

    def _key_press(self, event) -> None:
        if event.key and event.key.lower() == "z":
            self.zoom_between_cursors()
        elif event.key and event.key.lower() == "r":
            self.reset_view()

    def zoom_between_cursors(self, _event=None) -> None:
        left, right = sorted(self.cursor_times)
        if right - left < 1.0 / self.rate:
            return
        self.axes[0].set_xlim(left, right)

    def reset_view(self, _event=None) -> None:
        self.axes[0].set_xlim(0.0, self.duration)

    def show(self) -> None:
        self.plt.show()


def summarize(data: DataSet) -> None:
    duration = (len(data.samples) - 1) / data.sample_rate_hz
    minimum = np.min(data.samples, axis=0)
    maximum = np.max(data.samples, axis=0)
    print(
        f"{data.source}: {len(data.samples):,} samples, "
        f"{data.sample_rate_hz:g} samples/s, {duration:.9g} s"
    )
    print(
        f"range [{data.unit}]  "
        + "  ".join(
            f"{name}={low:+.7g}..{high:+.7g}"
            for name, low, high in zip("XYZ", minimum, maximum)
        )
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", nargs="?", type=Path, help="raw .npy or .npz file")
    parser.add_argument("--port", help="capture from the board, for example COM53")
    parser.add_argument(
        "--seconds", type=float, default=10.0,
        help="PSRAM recording duration (default: 10 seconds)",
    )
    parser.add_argument(
        "--rate", type=float, help="sample rate in Hz (capture default: 80000)"
    )
    parser.add_argument("--fs", type=int, choices=(50, 100, 200), default=50)
    parser.add_argument("--save", type=Path, help="capture output (default: raw_capture.npz)")
    parser.add_argument("--overwrite", action="store_true", help="replace capture output")
    parser.add_argument("--counts", action="store_true", help="display raw LSB counts")
    parser.add_argument("--unit", help="unit label for floating-point .npy input")
    parser.add_argument("--demo", action="store_true", help="open a synthetic cursor demo")
    parser.add_argument("--no-gui", action="store_true", help="print summary without plotting")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    source_count = int(args.file is not None) + int(args.port is not None) + int(args.demo)
    if source_count != 1:
        parser.error("choose exactly one source: FILE, --port, or --demo")
    if args.seconds <= 0:
        parser.error("--seconds must be greater than zero")
    if args.rate is not None and args.rate <= 0:
        parser.error("--rate must be greater than zero")

    try:
        if args.port:
            capture_rate = int(args.rate or 80000)
            if capture_rate not in ODR_CHOICES:
                parser.error("capture --rate must be one of " + ", ".join(map(str, ODR_CHOICES)))
            output = args.save or Path("raw_capture.npz")
            if output.suffix.lower() != ".npz":
                raise ValueError("capture output must use the .npz extension")
            if output.exists() and not args.overwrite:
                raise FileExistsError(f"{output} already exists; use --overwrite to replace it")
            capture = capture_serial(args.port, args.seconds, capture_rate, args.fs)
            counts = capture.samples_lsb
            np.savez_compressed(
                output,
                samples_lsb=counts.astype(np.int32, copy=False),
                sample_rate_hz=np.float64(capture.sample_rate_hz),
                configured_sample_rate_hz=np.int32(
                    capture.configured_sample_rate_hz
                ),
                capture_duration_s=np.float64(capture.capture_duration_s),
                full_scale_g=np.int16(capture.full_scale_g),
                output_bits=np.int16(capture.output_bits),
                overrun=np.bool_(capture.overrun),
                requested_duration_s=np.float64(args.seconds),
            )
            print(f"Saved exact raw counts and metadata to {output}.")
            if args.counts:
                data = DataSet(
                    counts, float(capture.sample_rate_hz), "LSB", str(output)
                )
            else:
                scale = MILLIG_PER_LSB[capture.full_scale_g] / 1000.0
                data = DataSet(
                    counts.astype(np.float64) * scale,
                    float(capture.sample_rate_hz),
                    "g",
                    str(output),
                )
        elif args.demo:
            data = demo_data(args.counts)
        else:
            data = load_file(args.file, args.rate, args.fs, args.counts, args.unit)

        summarize(data)
        if not args.no_gui:
            InteractiveInspector(data).show()
    except (FileNotFoundError, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
