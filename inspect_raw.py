#!/usr/bin/env python3
"""Capture and interactively inspect raw BowVib IMU samples.

Examples:
    python inspect_raw.py --port COM92
    python inspect_raw.py --wifi --seconds 5 --plot-png wifi.png --no-gui
    python inspect_raw.py raw_capture.npz
    python inspect_raw.py old_counts.npy --rate 80000 --fs 200
    python inspect_raw.py --demo

The plot has two draggable cursors. Their readout shows time, XYZ amplitude,
delta-time, amplitude differences, and 1/delta-time. Use the "Zoom A-B"
button (or press Z) to zoom to the interval between the cursors.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROW_BYTES = 6
SENSOR_COUNT = 4
MILLIG_PER_LSB = {50: 0.095, 100: 0.191, 200: 0.381, 320: 10.417}
HG_ODR_HZ = 7680
MAX_FRAME_ROWS = 256


def read_frame(read_exact_fn) -> tuple:
    """Read and validate one IM4 frame through either transport."""
    magic = read_exact_fn(4, timeout=15.0)
    if magic == b"IM4D":
        meta = read_exact_fn(3, timeout=3.0)
        sensor, rows = meta[0], struct.unpack_from("<H", meta, 1)[0]
        if sensor >= SENSOR_COUNT or not 1 <= rows <= MAX_FRAME_ROWS:
            raise RuntimeError(f"invalid data frame sensor={sensor} rows={rows}")
        return magic, sensor, decode_hg_samples(
            read_exact_fn(rows * ROW_BYTES, timeout=10.0)
        )
    if magic == b"IM4E":
        footer = read_exact_fn(SENSOR_COUNT * 4 + 5, timeout=3.0)
        counts = struct.unpack_from("<4I", footer, 0)
        duration_ms = struct.unpack_from("<I", footer, SENSOR_COUNT * 4)[0]
        if duration_ms == 0:
            raise RuntimeError("board reported a zero-length capture")
        return magic, counts, duration_ms / 1000.0, footer[-1]
    raise RuntimeError(f"invalid stream frame magic {magic!r}")


def archive_payload(arrays: tuple[np.ndarray, ...] | list[np.ndarray],
                    sensor_ids: tuple[int, ...], duration: float,
                    overrun_mask: int) -> dict[str, object]:
    """Keep full channels and include a common-length view for the inspector."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("capture duration must be finite and greater than zero")
    if not sensor_ids or len(set(sensor_ids)) != len(sensor_ids) or any(
            sensor_id not in range(1, SENSOR_COUNT + 1) for sensor_id in sensor_ids):
        raise ValueError("invalid sensor IDs")
    counts = np.asarray([len(values) for values in arrays], dtype=np.uint32)
    active = np.asarray(sensor_ids) - 1
    common = int(counts[active].min())
    if common < 2:
        raise ValueError("capture returned fewer than two samples per active IMU")
    rates = counts.astype(np.float64) / duration
    return {
        **{f"imu{i + 1}_samples_lsb": values for i, values in enumerate(arrays)},
        "samples_lsb": np.stack([arrays[i][:common] for i in active]),
        "sensor_counts": counts,
        "sensor_sample_rates_hz": rates,
        "sample_rate_hz": np.float64(rates[active].mean()),
        "sensor_ids": np.asarray(sensor_ids, dtype=np.uint8),
        "configured_sample_rate_hz": np.uint32(HG_ODR_HZ),
        "capture_duration_s": np.float64(duration),
        "full_scale_g": np.uint16(320),
        "output_bits": np.uint16(16),
        "overrun_mask": np.uint8(overrun_mask),
        "overrun_mask_valid": np.uint8(1),
    }


def sensor_label(sensor_id: int) -> str:
    letter = chr(ord("A") + sensor_id - 1)
    return f"Sensor {letter} (IMU {sensor_id})"


@dataclass
class DataSet:
    samples: np.ndarray
    sample_rate_hz: float
    unit: str
    source: str
    sensor_ids: tuple[int, ...] = (1,)


@dataclass
class CaptureResult:
    samples_lsb: np.ndarray  # equal-length view for plotting
    sensor_samples_lsb: tuple[np.ndarray, ...]  # full, untrimmed channels
    sample_rate_hz: float
    sensor_sample_rates_hz: np.ndarray
    configured_sample_rate_hz: int
    capture_duration_s: float
    full_scale_g: int
    output_bits: int
    sensor_counts: np.ndarray
    overrun_mask: int
    sensor_ids: tuple[int, ...]


def normalize_xyz(samples: np.ndarray) -> np.ndarray:
    """Validate Nx3 single-sensor or 4xNx3 multi-sensor samples."""
    values = np.asarray(samples)
    if values.ndim == 2:
        if values.shape[1] != 3 and values.shape[0] == 3:
            values = values.T
        if values.ndim != 2 or values.shape[1] != 3:
            raise ValueError(f"expected Nx3 XYZ samples, got shape {values.shape}")
        if len(values) < 2:
            raise ValueError("at least two samples are required")
    elif values.ndim == 3 and values.shape[0] > 0 and values.shape[2] == 3:
        if values.shape[1] < 2:
            raise ValueError("at least two samples per sensor are required")
    else:
        raise ValueError(f"expected Nx3 or 4xNx3 XYZ samples, got shape {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("sample data contains NaN or infinite values")
    return values


def decode_hg_samples(data: bytes | bytearray | memoryview) -> np.ndarray:
    """Decode little-endian signed 16-bit high-g XYZ samples."""
    raw = np.frombuffer(data, dtype="<i2")
    if raw.size % 3:
        raise ValueError("high-g data length is not a whole number of XYZ samples")
    return raw.reshape(-1, 3).astype(np.int32, copy=True)


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
            f"USB transfer stopped after {len(result):,}/{byte_count:,} bytes"
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
            value = token[len(prefix):]
            return int(value, 16) if key == "present_mask" else int(value)
    raise ValueError(f"board response is missing {key}=...: {line}")


def capture_serial(port: str, seconds: float) -> CaptureResult:
    """Stream detected high-g FIFO channels from the C6 to the host."""
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(
            "serial capture needs pyserial: python -m pip install pyserial"
        ) from exc

    serial_port = serial.Serial(port=None, baudrate=115200, timeout=0.1)
    serial_port.dtr = False
    serial_port.rts = False
    serial_port.port = port
    chunks: list[list[np.ndarray]] = [[] for _ in range(SENSOR_COUNT)]
    started_capture = False
    stop_sent = False
    try:
        serial_port.open()
        time.sleep(0.5)
        serial_port.reset_input_buffer()
        status = send_and_wait(serial_port, "STATUS", "STATUS")
        present_mask = line_integer(status, "present_mask")
        sensor_ids = tuple(i + 1 for i in range(SENSOR_COUNT)
                           if present_mask & (1 << i))
        if not sensor_ids:
            raise RuntimeError("board reports no responding IMUs")
        start = send_and_wait(serial_port, "START", "CAPTURE START")
        started_capture = True
        start_mask = line_integer(start, "present_mask")
        if start_mask != present_mask:
            raise RuntimeError(
                f"sensor presence changed between STATUS and START: "
                f"0x{present_mask:02X} -> 0x{start_mask:02X}"
            )
        print(f"Recording {', '.join(sensor_label(i) for i in sensor_ids)} at "
              f"{HG_ODR_HZ:,} samples/s, +/-320 g; streaming over USB ...")
        started = time.monotonic()
        stop_at = started + seconds
        stop_sent = False
        while True:
            if not stop_sent and time.monotonic() >= stop_at:
                serial_port.write(b"STOP\n")
                serial_port.flush()
                stop_sent = True
                print("  stopping and collecting FIFO tails ...")
            frame = read_frame(lambda count, timeout: read_exact(serial_port, count, timeout))
            if frame[0] == b"IM4D":
                if frame[1] + 1 not in sensor_ids:
                    raise RuntimeError("received data from an absent IMU")
                chunks[frame[1]].append(frame[2])
            else:
                sensor_counts = np.asarray(frame[1], dtype=np.uint32)
                duration_s, overrun_mask = frame[2:]
                break
    finally:
        if serial_port.is_open:
            if started_capture and not stop_sent:
                try:
                    serial_port.write(b"STOP\n")
                    serial_port.flush()
                except OSError:
                    pass
            serial_port.close()

    return finish_capture(chunks, sensor_ids, sensor_counts, duration_s, overrun_mask)


def read_socket_line(connection: socket.socket, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    line = bytearray()
    while time.monotonic() < deadline:
        connection.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            byte = connection.recv(1)
        except socket.timeout:
            continue
        if not byte:
            raise ConnectionError("Wi-Fi connection closed while waiting for a line")
        if byte in (b"\r", b"\n"):
            if line:
                return line.decode("ascii", errors="replace").strip()
        else:
            line.extend(byte)
    raise TimeoutError("timed out waiting for board text over Wi-Fi")


def read_socket_exact(connection: socket.socket, byte_count: int,
                       timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    result = bytearray()
    while len(result) < byte_count and time.monotonic() < deadline:
        connection.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            chunk = connection.recv(min(65536, byte_count - len(result)))
        except socket.timeout:
            continue
        if not chunk:
            raise ConnectionError(
                f"Wi-Fi stream closed after {len(result):,}/{byte_count:,} bytes"
            )
        result.extend(chunk)
    if len(result) != byte_count:
        raise TimeoutError(
            f"Wi-Fi transfer stopped after {len(result):,}/{byte_count:,} bytes"
        )
    return bytes(result)


def capture_wifi(host: str, port: int, seconds: float) -> CaptureResult:
    """Capture the same IM4 binary stream through the ESP32-C6 TCP access point."""
    chunks: list[list[np.ndarray]] = [[] for _ in range(SENSOR_COUNT)]
    stop_timer: threading.Timer | None = None
    stop_sent = threading.Event()
    with socket.create_connection((host, port), timeout=10.0) as connection:
        connection.settimeout(1.0)
        greeting = read_socket_line(connection, 10.0)
        print(f"  board> {greeting}")
        connection.sendall(b"STATUS\n")
        status = read_socket_line(connection, 4.0)
        print(f"  board> {status}")
        if status.startswith("ERR "):
            raise RuntimeError(status)
        present_mask = line_integer(status, "present_mask")
        sensor_ids = tuple(i + 1 for i in range(SENSOR_COUNT)
                           if present_mask & (1 << i))
        if not sensor_ids:
            raise RuntimeError("board reports no responding IMUs")

        connection.sendall(b"START\n")
        start = read_socket_line(connection, 4.0)
        print(f"  board> {start}")
        if start.startswith("ERR "):
            raise RuntimeError(start)
        if not start.startswith("CAPTURE START"):
            raise RuntimeError(f"unexpected start response: {start}")
        start_mask = line_integer(start, "present_mask")
        if start_mask != present_mask:
            raise RuntimeError(
                f"sensor presence changed between STATUS and START: "
                f"0x{present_mask:02X} -> 0x{start_mask:02X}"
            )
        print(f"Recording {', '.join(sensor_label(i) for i in sensor_ids)} at "
              f"{HG_ODR_HZ:,} samples/s, +/-320 g; streaming over Wi-Fi ...")

        def send_stop() -> None:
            try:
                connection.sendall(b"STOP\n")
                stop_sent.set()
            except OSError:
                pass

        stop_timer = threading.Timer(seconds, send_stop)
        stop_timer.daemon = True
        stop_timer.start()
        if seconds > 0.5:
            print("  capture in progress ...")
        try:
            while True:
                frame = read_frame(lambda count, timeout: read_socket_exact(connection, count, timeout))
                if frame[0] == b"IM4D":
                    if frame[1] + 1 not in sensor_ids:
                        raise RuntimeError("received data from an absent IMU")
                    chunks[frame[1]].append(frame[2])
                else:
                    sensor_counts = np.asarray(frame[1], dtype=np.uint32)
                    duration_s, overrun_mask = frame[2:]
                    break
        finally:
            if stop_timer is not None:
                stop_timer.cancel()
            if not stop_sent.is_set():
                send_stop()

    return finish_capture(chunks, sensor_ids, sensor_counts, duration_s, overrun_mask)


def finish_capture(chunks, sensor_ids, sensor_counts, duration_s, overrun_mask) -> CaptureResult:
    """Validate the footer and construct the same result for both transports."""
    arrays = [np.concatenate(part) if part else np.empty((0, 3), dtype=np.int32)
              for part in chunks]
    actual_counts = np.asarray([len(values) for values in arrays], dtype=np.uint32)
    if not np.array_equal(actual_counts, sensor_counts):
        raise RuntimeError(f"frame/footer count mismatch: frames={actual_counts}, footer={sensor_counts}")
    active_indices = np.asarray(sensor_ids, dtype=np.int32) - 1
    active_counts = actual_counts[active_indices]
    if np.any(active_counts == 0):
        raise RuntimeError("capture returned no samples from an active IMU")
    common_count = int(active_counts.min())
    if common_count < 2:
        raise RuntimeError("capture returned fewer than two samples per IMU")
    if np.any(active_counts != common_count):
        print(f"  sensor counts differ slightly; trimming each channel to {common_count:,} rows for plotting")
    samples = np.stack([arrays[i][:common_count] for i in active_indices])
    sensor_rates = actual_counts.astype(np.float64) / duration_s
    measured_rate = float(sensor_rates[active_indices].mean())
    fifo_overrun_mask = overrun_mask & 0x0F
    if fifo_overrun_mask:
        print(f"WARNING: sensor FIFO overrun mask=0x{fifo_overrun_mask:X} (bit 0 is IMU 1).")
    if overrun_mask & 0x80:
        print("WARNING: output transport overflow or send failure.")
    print("Samples per sensor: " + ", ".join(
        f"{sensor_label(i + 1)}={actual_counts[i]:,}" for i in active_indices))
    print(f"Capture duration {duration_s:.6f} s; mean effective rate {measured_rate:,.2f} samples/s per sensor.")
    return CaptureResult(samples, tuple(arrays), measured_rate, sensor_rates,
                         HG_ODR_HZ, duration_s, 320, 16, sensor_counts,
                         overrun_mask, sensor_ids)


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
    metadata_sensor_ids = None
    is_counts = False

    if path.suffix.lower() == ".npz":
        with np.load(path, allow_pickle=False) as archive:
            if "sensor_ids" in archive.files:
                metadata_sensor_ids = tuple(int(i) for i in np.asarray(archive["sensor_ids"]).reshape(-1))
            if "samples_lsb" in archive.files:
                samples = archive["samples_lsb"]
                is_counts = True
            elif "samples" in archive.files:
                samples = archive["samples"]
            elif "xyz" in archive.files:
                samples = archive["xyz"]
            elif "arr_0" in archive.files:
                samples = archive["arr_0"]
            elif any(f"imu{i}_samples_lsb" in archive.files for i in range(1, 5)):
                ids = metadata_sensor_ids or tuple(
                    i for i in range(1, 5) if f"imu{i}_samples_lsb" in archive.files
                    and len(archive[f"imu{i}_samples_lsb"]) > 0
                )
                arrays = [archive[f"imu{i}_samples_lsb"] for i in ids]
                if not arrays:
                    raise ValueError("archive contains no active IMU samples")
                common = min(len(values) for values in arrays)
                samples = np.stack([values[:common] for values in arrays])
                metadata_sensor_ids = ids
                is_counts = True
            else:
                raise ValueError(
                    f"{path} has no samples_lsb, samples, xyz, or arr_0 array"
                )
            metadata_rate = scalar(archive, "sample_rate_hz")
            if metadata_rate is None and "sensor_sample_rates_hz" in archive.files:
                if metadata_sensor_ids:
                    rates = np.asarray(archive["sensor_sample_rates_hz"])
                    metadata_rate = float(rates[np.asarray(metadata_sensor_ids) - 1].mean())
            metadata_fs = scalar(archive, "full_scale_g")
            metadata_unit = scalar(archive, "unit")
    elif path.suffix.lower() == ".npy":
        samples = np.load(path, allow_pickle=False)
        is_counts = np.issubdtype(samples.dtype, np.integer)
    else:
        raise ValueError("input must be a .npy or .npz file")

    samples = normalize_xyz(samples)
    rate = float(rate_override if rate_override is not None else
                 metadata_rate if metadata_rate is not None else HG_ODR_HZ)
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("sample rate must be greater than zero")
    if rate_override is None and metadata_rate is None:
        print("No sample-rate metadata found; using 7680 Hz; use --rate for legacy captures.")

    effective_fs = int(metadata_fs if metadata_fs is not None else full_scale)
    if effective_fs not in MILLIG_PER_LSB:
        raise ValueError(f"unsupported full-scale value: {effective_fs}")

    if is_counts and not show_counts:
        samples = samples.astype(np.float64) * (MILLIG_PER_LSB[effective_fs] / 1000.0)
        unit = "g"
    else:
        unit = unit_override or ("LSB" if is_counts else str(metadata_unit or "g"))
    sensor_count = samples.shape[0] if samples.ndim == 3 else 1
    sensor_ids = metadata_sensor_ids or tuple(range(1, sensor_count + 1))
    if len(sensor_ids) != sensor_count:
        raise ValueError("sensor_ids metadata does not match the sample channels")
    return DataSet(samples, rate, unit, str(path), sensor_ids)


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
        self.sensor_count = self.values.shape[0] if self.values.ndim == 3 else 1
        self.sensor_ids = data.sensor_ids
        self.sample_count = self.values.shape[1] if self.values.ndim == 3 else self.values.shape[0]
        self.duration = (self.sample_count - 1) / self.rate
        self.cursor_times = [self.duration / 3.0, self.duration * 2.0 / 3.0]
        self.dragging: int | None = None
        self._updating_view = False

        self.figure, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
        self.axes = list(axes)
        self.figure.canvas.manager.set_window_title("BowVib raw-data inspector")
        self.figure.subplots_adjust(top=0.88, bottom=0.23, hspace=0.08)
        self.figure.suptitle(
            f"{data.source} | {self.sample_count:,} samples x {self.sensor_count} IMUs | "
            f"{self.rate:g} samples/s"
        )

        axis_names = ("X", "Y", "Z")
        self.data_lines = []
        self.cursor_lines = [[], []]
        self.cursor_markers = [[[] for _ in range(3)] for _ in range(2)]
        cursor_colors = ("tab:red", "tab:purple")
        for axis_index, axis in enumerate(self.axes):
            axis_lines = []
            for sensor_index in range(self.sensor_count):
                line, = axis.plot([], [], linewidth=0.8,
                                  label=sensor_label(self.sensor_ids[sensor_index]))
                axis_lines.append(line)
            self.data_lines.append(axis_lines)
            if axis_index == 0:
                axis.legend(loc="upper right", ncol=4, fontsize=8)
            axis.set_ylabel(f"{axis_names[axis_index]} [{data.unit}]")
            axis.grid(True, alpha=0.25)
            for cursor_index, color in enumerate(cursor_colors):
                cursor_line = axis.axvline(
                    self.cursor_times[cursor_index], color=color, linewidth=1.2
                )
                markers = []
                for sensor_index in range(self.sensor_count):
                    marker, = axis.plot([], [], "o", color=color, markersize=5,
                                        markerfacecolor="none" if sensor_index else color)
                    markers.append(marker)
                self.cursor_lines[cursor_index].append(cursor_line)
                self.cursor_markers[cursor_index][axis_index] = markers
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
        return int(np.clip(round(time_seconds * self.rate), 0, self.sample_count - 1))

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
            last = min(self.sample_count, int(np.ceil(right * self.rate)) + 1)
            if last <= first:
                last = min(self.sample_count, first + 1)

            count = last - first
            if count > self.MAX_VISIBLE_POINTS:
                indices = np.linspace(
                    first, last - 1, self.MAX_VISIBLE_POINTS, dtype=np.int64
                )
                indices = np.unique(indices)
            else:
                indices = np.arange(first, last, dtype=np.int64)
            visible_time = indices.astype(np.float64) / self.rate

            segment = (self.values[:, first:last, :] if self.values.ndim == 3
                       else self.values[first:last, :])
            for axis_index, axis in enumerate(self.axes):
                for sensor_index in range(self.sensor_count):
                    series = (self.values[sensor_index, indices, axis_index]
                              if self.values.ndim == 3
                              else self.values[indices, axis_index])
                    self.data_lines[axis_index][sensor_index].set_data(visible_time, series)
                values = segment[:, :, axis_index] if self.values.ndim == 3 else segment[:, axis_index]
                low = float(np.min(values))
                high = float(np.max(values))
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
                self.cursor_lines[cursor_index][axis_index].set_xdata([cursor_time, cursor_time])
                for sensor_index in range(self.sensor_count):
                    value = (self.values[sensor_index, sample_index, axis_index]
                             if self.values.ndim == 3
                             else self.values[sample_index, axis_index])
                    self.cursor_markers[cursor_index][axis_index][sensor_index].set_data(
                        [cursor_time], [value])

        sample_a = (self.values[:, indices[0], :].astype(np.float64)
                    if self.values.ndim == 3 else self.values[indices[0]].astype(np.float64))
        sample_b = (self.values[:, indices[1], :].astype(np.float64)
                    if self.values.ndim == 3 else self.values[indices[1]].astype(np.float64))
        delta_t = abs(snapped_times[1] - snapped_times[0])
        frequency = np.inf if delta_t == 0.0 else 1.0 / delta_t
        delta_amplitude = sample_b - sample_a
        half_span = np.abs(delta_amplitude) / 2.0

        def xyz(values: np.ndarray) -> str:
            return "  ".join(f"{name}={value:+.7g}" for name, value in zip("XYZ", values))

        frequency_text = "infinite" if not np.isfinite(frequency) else f"{frequency:.7g} Hz"
        if self.values.ndim == 3:
            a_text = "\n".join(f"{sensor_label(self.sensor_ids[i])} A: {xyz(sample_a[i])}" for i in range(self.sensor_count))
            b_text = "\n".join(f"{sensor_label(self.sensor_ids[i])} B: {xyz(sample_b[i])}  delta: {xyz(delta_amplitude[i])}" for i in range(self.sensor_count))
            self.readout.set_text(
                f"A t={snapped_times[0]:.9f} s sample={indices[0]:,} [{self.data.unit}]\n"
                f"{a_text}\nB t={snapped_times[1]:.9f} s sample={indices[1]:,}  "
                f"delta-t={delta_t:.9g} s  1/delta-t={frequency_text} [{self.data.unit}]\n"
                f"{b_text}\nDrag either cursor; Z=zoom A-B; R=reset; wheel=zoom")
        else:
            self.readout.set_text(
                f"A: t={snapped_times[0]:.9f} s  sample={indices[0]:,}  {xyz(sample_a)} {self.data.unit}\n"
                f"B: t={snapped_times[1]:.9f} s  sample={indices[1]:,}  {xyz(sample_b)} {self.data.unit}\n"
                f"delta-t={delta_t:.9g} s   1/delta-t={frequency_text}   "
                f"delta: {xyz(delta_amplitude)} {self.data.unit}\n"
                f"half |delta| (peak estimate): {xyz(half_span)} {self.data.unit}   "
                "Drag either cursor; Z=zoom A-B; R=reset; wheel=zoom")
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
    sample_count = data.samples.shape[1] if data.samples.ndim == 3 else len(data.samples)
    duration = (sample_count - 1) / data.sample_rate_hz
    minimum = np.min(data.samples, axis=1 if data.samples.ndim == 3 else 0)
    maximum = np.max(data.samples, axis=1 if data.samples.ndim == 3 else 0)
    sensor_note = f" x {data.samples.shape[0]} IMUs" if data.samples.ndim == 3 else ""
    print(f"{data.source}: {sample_count:,} samples{sensor_note}, "
          f"{data.sample_rate_hz:g} samples/s, {duration:.9g} s")
    if data.samples.ndim == 3:
        for sensor in range(data.samples.shape[0]):
            print(f"{sensor_label(data.sensor_ids[sensor])} range [{data.unit}]  " + "  ".join(
                f"{name}={low:+.7g}..{high:+.7g}"
                for name, low, high in zip("XYZ", minimum[sensor], maximum[sensor])))
    else:
        print(f"range [{data.unit}]  " + "  ".join(
            f"{name}={low:+.7g}..{high:+.7g}"
            for name, low, high in zip("XYZ", minimum, maximum)))


def save_plot(data: DataSet, output: Path) -> None:
    """Save a static three-axis plot without requiring a desktop GUI."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "plot export needs Matplotlib: python -m pip install matplotlib"
        ) from exc

    values = normalize_xyz(data.samples)
    count = values.shape[1] if values.ndim == 3 else values.shape[0]
    indices = np.arange(count)
    if count > InteractiveInspector.MAX_VISIBLE_POINTS:
        indices = np.linspace(
            0, count - 1, InteractiveInspector.MAX_VISIBLE_POINTS, dtype=np.int64
        )
    times = indices.astype(np.float64) / data.sample_rate_hz
    figure, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for axis_index, axis in enumerate(axes):
        for sensor_index, sensor_id in enumerate(data.sensor_ids):
            series = (values[sensor_index, indices, axis_index]
                      if values.ndim == 3 else values[indices, axis_index])
            axis.plot(times, series, linewidth=0.8, label=sensor_label(sensor_id))
        axis.set_ylabel(f"{'XYZ'[axis_index]} [{data.unit}]")
        axis.grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    figure.suptitle(
        f"{data.source} | {count:,} samples | {data.sample_rate_hz:g} samples/s"
    )
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", nargs="?", type=Path, help="raw .npy or .npz file")
    parser.add_argument("--port", help="capture from the board, for example COM92")
    parser.add_argument(
        "--wifi", metavar="HOST", nargs="?", const="192.168.4.1",
        help="capture over Wi-Fi from the board (default host 192.168.4.1)",
    )
    parser.add_argument("--wifi-port", type=int, default=3333,
                        help="Wi-Fi TCP port (default: 3333)")
    parser.add_argument(
        "--seconds", type=float, default=10.0,
        help="live capture duration (default: 10 seconds)",
    )
    parser.add_argument("--rate", type=float, help="capture is fixed at 7680 Hz per sensor")
    parser.add_argument("--fs", type=int, choices=(50, 100, 200, 320), default=320)
    parser.add_argument("--save", type=Path, help="capture output (default: raw_capture.npz)")
    parser.add_argument("--overwrite", action="store_true", help="replace capture output")
    parser.add_argument("--counts", action="store_true", help="display raw LSB counts")
    parser.add_argument("--unit", help="unit label for floating-point .npy input")
    parser.add_argument("--demo", action="store_true", help="open a synthetic cursor demo")
    parser.add_argument("--plot-png", type=Path, help="save a static plot as PNG")
    parser.add_argument("--no-gui", action="store_true", help="print summary without plotting")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    source_count = (int(args.file is not None) + int(args.port is not None) +
                    int(args.wifi is not None) + int(args.demo))
    if source_count != 1:
        parser.error("choose exactly one source: FILE, --port, --wifi, or --demo")
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be greater than zero")
    if args.rate is not None and (not math.isfinite(args.rate) or args.rate <= 0):
        parser.error("--rate must be greater than zero")

    try:
        if args.port or args.wifi is not None:
            capture_rate = args.rate or HG_ODR_HZ
            if capture_rate != HG_ODR_HZ:
                parser.error(f"capture rate is fixed at {HG_ODR_HZ} Hz per sensor")
            output = args.save or Path("raw_capture.npz")
            if output.suffix.lower() != ".npz":
                raise ValueError("capture output must use the .npz extension")
            if output.exists() and not args.overwrite:
                raise FileExistsError(f"{output} already exists; use --overwrite to replace it")
            if args.fs != 320:
                parser.error("capture full scale is fixed at 320 g")
            capture = (capture_serial(args.port, args.seconds) if args.port else
                       capture_wifi(args.wifi, args.wifi_port, args.seconds))
            counts = capture.samples_lsb
            np.savez_compressed(
                output,
                **archive_payload(capture.sensor_samples_lsb, capture.sensor_ids,
                                  capture.capture_duration_s, capture.overrun_mask),
                requested_duration_s=np.float64(args.seconds),
            )
            print(f"Saved exact raw counts and metadata to {output}.")
            if args.counts:
                data = DataSet(
                    counts, float(capture.sample_rate_hz), "LSB", str(output),
                    capture.sensor_ids,
                )
            else:
                scale = MILLIG_PER_LSB[capture.full_scale_g] / 1000.0
                data = DataSet(
                    counts.astype(np.float64) * scale,
                    float(capture.sample_rate_hz),
                    "g",
                    str(output),
                    capture.sensor_ids,
                )
        elif args.demo:
            data = demo_data(args.counts)
        else:
            data = load_file(args.file, args.rate, args.fs, args.counts, args.unit)

        summarize(data)
        if args.plot_png:
            save_plot(data, args.plot_png)
            print(f"Saved plot to {args.plot_png}.")
        if not args.no_gui:
            InteractiveInspector(data).show()
    except (FileNotFoundError, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
