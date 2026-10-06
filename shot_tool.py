#!/usr/bin/env python3
"""Control the on-board recorder, validate BVR1, export and compare shots."""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import struct
import time
import urllib.parse
import urllib.request
import zlib

import numpy as np

MANIFEST = struct.Struct("<4s11I3Q16s4b512sI")
PAGE = struct.Struct("<4sIIIQQ")
PAGE_BYTES = 4096
SENSITIVITY_G = 0.010417


@dataclass
class Shot:
    metadata: dict
    arrays: list[np.ndarray]
    times: list[np.ndarray]
    drain_times: list[np.ndarray]
    indices: list[np.ndarray]


def read_shot(path: Path) -> Shot:
    """Reject corrupt, truncated, reordered, or internally inconsistent recordings."""
    with path.open("rb") as source:
        header = source.read(MANIFEST.size)
        if len(header) != MANIFEST.size:
            raise ValueError("Truncated manifest")
        fields = MANIFEST.unpack(header)
        magic, version, first, end, capacity, pre, post, rate, fs, present, quality, saturation = fields[:12]
        started, trigger, ended, shot_id = fields[12:16]
        trims, raw_metadata, crc = fields[16:20], fields[20], fields[21]
        if magic != b"BVR1" or version not in (1, 2) or zlib.crc32(header[:-4]) != crc:
            raise ValueError("Invalid manifest or CRC")
        if not (0 <= first < end and 0 < end - first <= capacity <= 2048):
            raise ValueError("Invalid page range")
        if rate != 7680 or fs != 320 or not 0 < present < 16 or pre > 10000 or post > 3000 or pre + post == 0:
            raise ValueError("Unsupported acquisition configuration")
        if ended < started or (trigger and not started <= trigger <= ended) or b"\0" not in raw_metadata:
            raise ValueError("Invalid timestamps or metadata termination")
        metadata_text = raw_metadata.split(b"\0", 1)[0].decode("utf-8")
        try:
            test_metadata = json.loads(metadata_text)
        except json.JSONDecodeError:
            test_metadata = {"raw_text": metadata_text}
        metadata = dict(id=shot_id.hex(), pre_ms=pre, post_ms=post, odr_hz=rate,
                        fs_g=fs, present_mask=present, quality_mask=quality,
                        saturation_mask=saturation, started_us=started,
                        trigger_us=trigger, triggered=bool(trigger), ended_us=ended, freq_fine=list(trims),
                        test=test_metadata, timing="Estimated by fitting software FIFO drain timestamps to sample indices; no verified hardware sync")
        trigger_info = {"kind": "manual"} if version == 2 and trigger else None
        chunks, indices, drains = [[] for _ in range(4)], [[] for _ in range(4)], [[] for _ in range(4)]
        next_index = [None] * 4
        last_data_record = None
        last_page_time = 0
        for seq in range(first, end):
            page = source.read(PAGE_BYTES)
            if len(page) != PAGE_BYTES:
                raise ValueError("Truncated data page")
            pmagic, pseq, used, pcrc, first_us, last_us = PAGE.unpack_from(page)
            if pmagic != b"BVP1" or pseq != seq or not PAGE.size < used <= PAGE_BYTES:
                raise ValueError("Invalid page header or sequence")
            checked = bytearray(page[:used]); checked[12:16] = b"\0" * 4
            if zlib.crc32(checked) != pcrc:
                raise ValueError("Data page CRC mismatch")
            if not started <= first_us <= last_us <= ended or first_us < last_page_time:
                raise ValueError("Non-monotonic page timestamps")
            offset, record_times = PAGE.size, []
            while offset < used:
                if used - offset < 19:
                    raise ValueError("Truncated sample record")
                drain, index, frame, sensor, rows = struct.unpack_from("<QI4sBH", page, offset)
                if not first_us <= drain <= last_us or (record_times and drain < record_times[-1]):
                    raise ValueError("Non-monotonic sample timestamps")
                if frame == b"IM4T":
                    if version != 2 or not trigger or rows or sensor >= 4 or not present & (1 << sensor) or offset + 23 > used or trigger_info["kind"] != "manual":
                        raise ValueError("Invalid threshold trigger record")
                    threshold = struct.unpack_from("<I", page, offset + 19)[0]
                    if not 20 <= threshold <= 320000:
                        raise ValueError("Invalid threshold trigger value")
                    if last_data_record is None or last_data_record[0] != sensor or last_data_record[3] != drain or not last_data_record[1] <= index < last_data_record[1] + last_data_record[2]:
                        raise ValueError("Invalid threshold trigger sample reference")
                    trigger_info = dict(kind="threshold", sensor=sensor, index=index, thresholdMg=threshold)
                    record_times.append(drain); offset += 23
                    continue
                length = 19 + rows * 6
                if frame != b"IM4D" or sensor >= 4 or not present & (1 << sensor) or not 1 <= rows <= 256 or offset + length > used:
                    raise ValueError("Invalid sample frame")
                if next_index[sensor] is not None and index != next_index[sensor]:
                    raise ValueError("Discontinuous per-sensor sample sequence")
                next_index[sensor] = index + rows
                last_data_record = (sensor, index, rows, drain)
                raw = np.frombuffer(page, dtype="<i2", count=rows * 3, offset=offset + 19).reshape(-1, 3).copy()
                chunks[sensor].append(raw)
                indices[sensor].append(np.arange(index, index + rows, dtype=np.uint64))
                drains[sensor].append(np.full(rows, (drain - (trigger or started)) / 1e6))
                record_times.append(drain); offset += length
            if record_times[0] != first_us or record_times[-1] != last_us:
                raise ValueError("Page timestamp bounds disagree with records")
            last_page_time = last_us
        if source.read(1):
            raise ValueError("Unexpected data after final page")
    arrays, times, kept_drains, kept_indices = [], [], [], []
    fitted, full_values, full_indices, full_drains = [], [], [], []
    warnings, fitted_rates, timing_residuals = [], [], []
    for sensor in range(4):
        values = np.concatenate(chunks[sensor]) if chunks[sensor] else np.empty((0, 3), dtype=np.int16)
        idx = np.concatenate(indices[sensor]) if indices[sensor] else np.empty(0, dtype=np.uint64)
        drain = np.concatenate(drains[sensor]) if drains[sensor] else np.empty(0)
        trim = trims[sensor]
        predicted = rate * (1 if trim == -128 else 1 + trim * 0.0013)
        estimated_rate, residual = predicted, None
        slope, intercept = 1 / predicted, (started - (trigger or started)) / 1e6
        if len(idx) >= 2:
            # A FIFO drain timestamp belongs to the batch, not each sample.
            # Fit only batch endpoints, then use a median intercept to reduce latency outliers.
            ends = np.r_[np.diff(drain) != 0, True]
            x, y = idx[ends].astype(np.float64), drain[ends]
            dx, dy = x - x.mean(), y - y.mean()
            denom = float(np.dot(dx, dx))
            if denom > 0:
                measured_slope = float(np.dot(dx, dy) / denom)
                if measured_slope > 0:
                    slope = measured_slope
                    estimated_rate = 1 / slope
                    intercept = float(np.median(y - slope * x))
                    residual = float(np.std(y - (slope * x + intercept)))
                    if not .8 * rate < estimated_rate < 1.2 * rate:
                        warnings.append(f"Sensor {sensor + 1}: estimated rate outside plausible range")
        fitted_rates.append(estimated_rate); timing_residuals.append(residual)
        fitted.append((slope, intercept))
        full_values.append(values); full_indices.append(idx); full_drains.append(drain)
    # The software trigger timestamp is only an estimate of the sensor clock.
    # For new automatic captures, anchor the common time origin to the recorded
    # threshold sample, after fitting all sensors, rather than the FIFO callback.
    alignment_shift = 0.0
    if trigger_info and trigger_info["kind"] == "threshold":
        sensor, index = trigger_info["sensor"], trigger_info["index"]
        idx = full_indices[sensor]
        if not len(idx) or not int(idx[0]) <= index <= int(idx[-1]):
            raise ValueError("Threshold trigger sample is missing")
        slope, intercept = fitted[sensor]
        alignment_shift = intercept + index * slope
        metadata["timing"] = "Zero is the recorded threshold sample; other sample times estimated by fitting software FIFO drain timestamps; no verified hardware sync"
    metadata["trigger_info"] = trigger_info
    metadata["trigger_alignment_shift_s"] = alignment_shift
    window_start = -pre / 1000 if trigger else 0
    window_end = post / 1000 if trigger else (ended - started) / 1e6
    for sensor in range(4):
        values, idx, drain = full_values[sensor], full_indices[sensor], full_drains[sensor] - alignment_shift
        slope, intercept = fitted[sensor]
        if trigger_info and trigger_info["kind"] == "threshold" and trigger_info["sensor"] == sensor:
            t = (idx.astype(np.float64) - trigger_info["index"]) * slope
        else:
            t = idx.astype(np.float64) * slope + (intercept - alignment_shift)
        # Timestamp quantization is 1 us; retain samples lying on a rounded boundary.
        keep = (t >= window_start - 1e-6) & (t <= window_end + 1e-6)
        values, t, idx, drain = values[keep], t[keep], idx[keep], drain[keep]
        arrays.append(values); times.append(t); kept_indices.append(idx); kept_drains.append(drain)
        if present & (1 << sensor):
            if len(t) < 2:
                warnings.append(f"Sensor {sensor + 1}: missing samples in window")
            elif t[0] > window_start + 0.04 or t[-1] < window_end - 0.04:
                warnings.append(f"Sensor {sensor + 1}: requested window coverage incomplete")
    if quality:
        warnings.append(f"Firmware quality mask 0x{quality:02x}")
    if saturation:
        warnings.append(f"Saturation detected during armed session: mask 0x{saturation:02x}")
    metadata["warnings"] = warnings
    metadata["valid"] = not warnings
    metadata["sensor_counts"] = [len(a) for a in arrays]
    metadata["sensor_estimated_sample_rates_hz"] = fitted_rates
    metadata["sensor_timing_residual_std_s"] = timing_residuals
    return Shot(metadata, arrays, times, kept_drains, kept_indices)


def load_calibration(path: Path | None) -> dict:
    if path is None:
        return {"offsets_g": [[0, 0, 0]] * 4, "rotations": [np.eye(3).tolist()] * 4}
    calibration = json.loads(path.read_text(encoding="utf-8"))
    offsets, rotations = np.asarray(calibration["offsets_g"]), np.asarray(calibration["rotations"])
    if offsets.shape != (4, 3) or rotations.shape != (4, 3, 3) or not np.isfinite(offsets).all() or not np.isfinite(rotations).all():
        raise ValueError("Calibration needs four finite XYZ offsets and four 3x3 rotations")
    for rotation in rotations:
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6):
            raise ValueError("Orientation matrices must be proper orthonormal rotations")
    return calibration


def processed(shot: Shot, calibration: dict) -> list[np.ndarray]:
    return [(values.astype(np.float64) * SENSITIVITY_G - calibration["offsets_g"][i]) @ np.asarray(calibration["rotations"][i]).T
            for i, values in enumerate(shot.arrays)]


def check_outputs(paths: list[Path], overwrite: bool) -> None:
    existing = [str(p) for p in paths if p.exists()]
    if existing and not overwrite:
        raise FileExistsError("Already exists: " + ", ".join(existing))


def export_shot(shot: Shot, directory: Path, calibration: dict, overwrite: bool = False) -> None:
    stem = "shot_" + shot.metadata["id"]
    npz, csv_path, json_path = [directory / (stem + suffix) for suffix in (".npz", ".csv", ".json")]
    check_outputs([npz, csv_path, json_path], overwrite)
    directory.mkdir(parents=True, exist_ok=True)
    ids = [i + 1 for i in range(4) if shot.metadata["present_mask"] & (1 << i)]
    meta = {**shot.metadata, "calibration": calibration}
    g_arrays = processed(shot, calibration)
    payload = {f"imu{i + 1}_samples_lsb": a for i, a in enumerate(shot.arrays)}
    payload.update({f"imu{i + 1}_time_s": t for i, t in enumerate(shot.times)})
    payload.update({f"imu{i + 1}_sample_indices": t for i, t in enumerate(shot.indices)})
    payload.update({f"imu{i + 1}_drain_time_s": t for i, t in enumerate(shot.drain_times)})
    payload.update({f"imu{i + 1}_processed_g": a for i, a in enumerate(g_arrays)})
    # Report the retained data span, including full no-trigger sessions and early pulses.
    spans = [(t[0], t[-1] + 1 / shot.metadata["sensor_estimated_sample_rates_hz"][i])
             for i, t in enumerate(shot.times) if len(t)]
    duration = max(end for _, end in spans) - min(start for start, _ in spans) if spans else 0
    fine = shot.metadata["freq_fine"]
    rates = np.array(shot.metadata["sensor_estimated_sample_rates_hz"])
    payload["sensor_expected_sample_rates_hz"] = np.array([7680 * (1 if v == -128 else 1 + v * .0013) for v in fine])
    payload.update(sensor_ids=np.array(ids, dtype=np.uint8), sensor_counts=np.array(shot.metadata["sensor_counts"]),
                   sensor_sample_rates_hz=rates, sample_rate_hz=float(rates[np.array(ids) - 1].mean()),
                   configured_sample_rate_hz=7680, full_scale_g=320, output_bits=16,
                   capture_duration_s=duration, overrun_mask=shot.metadata["quality_mask"],
                   overrun_mask_valid=1, shot_metadata_json=json.dumps(meta, ensure_ascii=False))
    np.savez_compressed(npz, **payload)
    with csv_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(["shot_id", "sensor", "sample_index", "estimated_time_s", "drain_time_s", "x_lsb", "y_lsb", "z_lsb", "x_g", "y_g", "z_g", "resultant_g"])
        for i in range(4):
            for idx, t, drain, raw, values in zip(shot.indices[i], shot.times[i], shot.drain_times[i], shot.arrays[i], g_arrays[i]):
                writer.writerow([shot.metadata["id"], i + 1, int(idx), t, drain, *raw, *values, float(np.linalg.norm(values))])
    json_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")


def analyze(shot: Shot, calibration: dict, output: Path, overwrite: bool = False) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    report_path = output.with_suffix(".json")
    if report_path == output:
        raise ValueError("Plot output must have an image extension")
    check_outputs([output, report_path], overwrite)
    output.parent.mkdir(parents=True, exist_ok=True)
    ids = [i for i in range(4) if len(shot.arrays[i]) >= 2]
    if not ids:
        raise ValueError("No sensor has enough data for analysis")
    fig, axes = plt.subplots(len(ids), 3, figsize=(16, 3.5 * len(ids)), squeeze=False)
    stats = []
    for row, i in enumerate(ids):
        xyz, t = processed(shot, calibration)[i], shot.times[i]
        dt = float(np.median(np.diff(t))); rate = 1 / dt
        resultant = np.linalg.norm(xyz, axis=1)
        rms_window = min(len(resultant), max(1, round(rate * .025)))
        # RMS envelope of the vibration component (remove static gravity and offset).
        vibration = xyz - xyz.mean(axis=0)
        envelope = np.sqrt(np.convolve(np.sum(vibration ** 2, axis=1), np.ones(rms_window) / rms_window, mode="same"))
        for axis in range(3):
            axes[row, 0].plot(t, xyz[:, axis], label="XYZ"[axis], linewidth=.7)
        axes[row, 0].set_title(f"IMU {i + 1} XYZ"); axes[row, 0].legend(); axes[row, 0].set_ylabel("g")
        axes[row, 1].plot(t, resultant, label="Resultant", linewidth=.7)
        axes[row, 1].plot(t, envelope, label="25 ms RMS envelope", linewidth=.8)
        axes[row, 1].legend(); axes[row, 1].set_ylabel("g")
        # Analyze the post-trigger event when available; reference shots use the full window.
        event = vibration[t >= 0] if np.count_nonzero(t >= 0) >= 8 else vibration
        window = np.hanning(len(event))
        spectrum = np.abs(np.fft.rfft(event * window[:, None], axis=0)) * (2 / max(window.sum(), 1))
        spectrum[0] *= .5
        if len(event) % 2 == 0: spectrum[-1] *= .5
        frequency = np.fft.rfftfreq(len(event), dt)
        for axis in range(3):
            axes[row, 2].plot(frequency, spectrum[:, axis], label="XYZ"[axis], linewidth=.8)
        axes[row, 2].set_title("Mean-removed Hann FFT amplitude"); axes[row, 2].set_xlabel("Hz"); axes[row, 2].set_ylabel("g")
        for panel in axes[row, :2]:
            panel.axvline(0, color="black", linestyle="--", linewidth=.6); panel.set_xlabel("Estimated time from trigger [s]")
        post = t >= 0
        measured = xyz[post] if np.any(post) else xyz
        stats.append({"sensor": i + 1, "peak_resultant_g": float(np.linalg.norm(measured, axis=1).max()),
                      "rms_resultant_g": float(np.sqrt(np.mean(np.sum(measured ** 2, axis=1))))})
    fig.suptitle(f"Shot {shot.metadata['id']} | {'VALIDATION REQUIRED' if shot.metadata['valid'] else 'REVIEW: ' + '; '.join(shot.metadata['warnings'])}", fontsize=10)
    fig.tight_layout(); fig.savefig(output, dpi=160); plt.close(fig)
    report_path.write_text(json.dumps({"shot": shot.metadata, "statistics": stats, "calibration": calibration}, indent=2), encoding="utf-8")


def compare(shots: list[Shot], calibration: dict, output: Path, overwrite: bool = False) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    report_path = output.with_suffix(".json")
    if output == report_path:
        raise ValueError("Plot output must have an image extension")
    check_outputs([output, report_path], overwrite)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, 1, figsize=(12, 12))
    groups = {}
    for shot in shots:
        configuration = json.dumps(shot.metadata["test"], sort_keys=True, ensure_ascii=False)
        values = processed(shot, calibration)
        for i in range(4):
            if len(values[i]) < 2:
                continue
            resultant = np.linalg.norm(values[i], axis=1)
            axes[i].plot(shot.times[i], resultant, label=shot.metadata["id"][:8] + (" REVIEW" if not shot.metadata["valid"] else ""), linewidth=.7)
            post = resultant[shot.times[i] >= 0]
            if shot.metadata["valid"] and len(post):
                groups.setdefault((configuration, i + 1), []).append(float(post.max()))
    summary = []
    for (configuration, sensor), peaks in groups.items():
        summary.append({"test_metadata": json.loads(configuration), "sensor": sensor, "valid_shots": len(peaks),
                        "mean_peak_g": float(np.mean(peaks)), "std_peak_g": float(np.std(peaks, ddof=1)) if len(peaks) > 1 else None})
    for i, axis in enumerate(axes):
        axis.set_title(f"IMU {i + 1} resultant"); axis.set_ylabel("g"); axis.set_xlabel("Estimated time from trigger [s]")
        axis.axvline(0, color="black", linestyle="--", linewidth=.6)
        if axis.lines[:-1]: axis.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(output, dpi=160); plt.close(fig)
    report_path.write_text(json.dumps({"shots": [s.metadata for s in shots], "repeat_statistics": summary}, indent=2), encoding="utf-8")


def api(host: str, path: str, data: bytes | None = None) -> bytes:
    request = urllib.request.Request(f"http://{host}{path}", data=data,
                                    headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def usb_api(port: str, command: str) -> bytes:
    import serial
    connection = serial.Serial(port=None, baudrate=115200, timeout=.2, write_timeout=3)
    connection.dtr = False; connection.rts = False; connection.port = port
    connection.open()
    with connection:
        connection.reset_input_buffer()
        connection.write((command + "\n").encode("utf-8"))
        deadline, line = time.monotonic() + 120, bytearray()
        while time.monotonic() < deadline:
            byte = connection.read(1)
            if byte in (b"\r", b"\n"):
                text = line.decode("ascii", errors="replace"); line.clear()
                if text.startswith("ERR "): raise ValueError(text)
                if text.startswith("SHOT DATA bytes="):
                    size = int(text.split("=")[1])
                    if not MANIFEST.size <= size <= 8 * 1024 * 1024: raise ValueError("Invalid download size")
                    result = bytearray()
                    while len(result) < size and time.monotonic() < deadline:
                        result.extend(connection.read(min(65536, size - len(result))))
                    if len(result) != size: raise ValueError("Truncated USB shot download")
                    return bytes(result)
                if text.startswith("SHOT "): return text.encode()
            elif byte:
                line.extend(byte)
                if len(line) > 1024: raise ValueError("Invalid USB response")
        raise TimeoutError("USB shot command timed out")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", help="Use local USB commands instead of Wi-Fi, e.g. COM92")
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("status", "trigger", "cancel", "delete"):
        sub.add_parser(name)
    arm = sub.add_parser("arm")
    arm.add_argument("--pre", type=float, default=10); arm.add_argument("--post", type=float, default=3)
    arm.add_argument("--threshold-g", type=float, default=50)
    arm.add_argument("--metadata", type=Path, help="JSON object with test configuration")
    fetch = sub.add_parser("download"); fetch.add_argument("output", type=Path); fetch.add_argument("--overwrite", action="store_true")
    for name in ("export", "analyze", "calibrate"):
        command = sub.add_parser(name); command.add_argument("file", type=Path); command.add_argument("output", type=Path)
        command.add_argument("--overwrite", action="store_true")
        if name != "calibrate": command.add_argument("--calibration", type=Path)
        else: command.add_argument("--gravity", nargs=3, type=float, default=[0, 0, 1], help="Known gravity XYZ in each sensor's local axes [g]")
    comparison = sub.add_parser("compare"); comparison.add_argument("output", type=Path); comparison.add_argument("files", type=Path, nargs="+")
    comparison.add_argument("--calibration", type=Path); comparison.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    try:
        if args.command == "status": print((usb_api(args.port, "SHOTSTATUS") if args.port else api(args.host, "/shot")).decode())
        elif args.command in ("trigger", "cancel", "delete"):
            print((usb_api(args.port, "SHOT" + args.command.upper()) if args.port else api(args.host, "/shot/" + args.command, b"{}")).decode())
        elif args.command == "arm":
            values = {"pre_ms": args.pre * 1000, "post_ms": args.post * 1000, "threshold_mg": args.threshold_g * 1000}
            limits = {"pre_ms": 10000, "post_ms": 3000, "threshold_mg": 320000}
            if any(not np.isfinite(v) or v != round(v) or not 0 <= v <= limits[k] for k, v in values.items()):
                raise ValueError("Pre 0–10 s, post 0–3 s, threshold 0–320 g; millisecond/milligravity resolution")
            meta = json.loads(args.metadata.read_text(encoding="utf-8")) if args.metadata else {}
            if not isinstance(meta, dict): raise ValueError("Metadata must be a JSON object")
            data = json.dumps(meta, ensure_ascii=False).encode()
            if len(data) >= 512: raise ValueError("Metadata must be under 512 UTF-8 bytes")
            query = urllib.parse.urlencode({k: int(v) for k, v in values.items()})
            command = "SHOTARM " + " ".join(str(int(values[k])) for k in ("pre_ms", "post_ms", "threshold_mg"))
            if args.metadata: command += " " + data.decode("utf-8")
            result = usb_api(args.port, command) if args.port else api(args.host, "/shot/arm?" + query, data)
            print(result.decode())
        elif args.command == "download":
            check_outputs([args.output], args.overwrite)
            data = usb_api(args.port, "SHOTGET") if args.port else api(args.host, "/shot.bin")
            args.output.parent.mkdir(parents=True, exist_ok=True); args.output.write_bytes(data)
            shot = read_shot(args.output); print(json.dumps(shot.metadata, indent=2))
        elif args.command == "compare":
            compare([read_shot(p) for p in args.files], load_calibration(args.calibration), args.output, args.overwrite)
        else:
            shot = read_shot(args.file)
            if args.command == "export": export_shot(shot, args.output, load_calibration(args.calibration), args.overwrite)
            elif args.command == "analyze": analyze(shot, load_calibration(args.calibration), args.output, args.overwrite)
            else:
                check_outputs([args.output], args.overwrite)
                if not shot.metadata["valid"]: raise ValueError("Cannot calibrate from a shot with quality warnings")
                gravity = np.asarray(args.gravity)
                if not np.isfinite(gravity).all() or not np.isclose(np.linalg.norm(gravity), 1, atol=.05):
                    raise ValueError("Gravity vector must have magnitude approximately 1 g")
                offsets, deviations = [], []
                for i, values in enumerate(shot.arrays):
                    if shot.metadata["present_mask"] & (1 << i) and (len(values) < 2 or np.ptp(shot.times[i]) < 9.9):
                        raise ValueError("Calibration requires at least 10 s of stationary data for each present sensor")
                    g = values.astype(float) * SENSITIVITY_G
                    offsets.append((g.mean(axis=0) - gravity).tolist() if len(g) else [0, 0, 0])
                    deviations.append(g.std(axis=0).tolist() if len(g) else None)
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps({"source_shot_id": shot.metadata["id"], "offsets_g": offsets,
                    "rotations": [np.eye(3).tolist()] * 4, "stationary_std_g": deviations,
                    "gravity_local_g": gravity.tolist(), "scope": "Offset verification only; scale and orientation require documented fixture tests"}, indent=2), encoding="utf-8")
            print(json.dumps(shot.metadata, indent=2))
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
