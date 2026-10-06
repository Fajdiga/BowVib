import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

import numpy as np

from shot_tool import MANIFEST, PAGE, PAGE_BYTES, read_shot, export_shot, load_calibration, analyze, compare


def fixture(*, quality=0, saturation=0, first=1500, trim=0, total=23040):
    """Build a four-channel recording spanning a flash ring wrap and a trigger."""
    pages, payload, times = [], bytearray(), []
    rate, rows = 7680, 128

    def flush():
        nonlocal payload, times
        page = bytearray(b"\xff" * PAGE_BYTES)
        PAGE.pack_into(page, 0, b"BVP1", first + len(pages), PAGE.size + len(payload), 0, times[0], times[-1])
        page[PAGE.size:PAGE.size + len(payload)] = payload
        struct.pack_into("<I", page, 12, zlib.crc32(page[:PAGE.size + len(payload)]))
        pages.append(page); payload, times = bytearray(), []

    for index in range(0, total, rows):
        drain = 1000000 + round((index + rows - 1) * 1e6 / rate)
        for sensor in range(4):
            values = np.zeros((rows, 3), dtype="<i2")
            values[:, 0] = np.rint(20 * np.sin(2 * np.pi * (np.arange(index, index + rows) / rate) * 200))
            values[:, 2] = 96
            record = struct.pack("<QI4sBH", drain, index, b"IM4D", sensor, rows) + values.tobytes()
            if PAGE.size + len(payload) + len(record) > PAGE_BYTES:
                flush()
            payload.extend(record); times.append(drain)
    if payload: flush()
    fields = (b"BVR1", 1, first, first + len(pages), 1519, 1000, 2000, rate, 320, 15,
              quality, saturation, 1000000, 2000000, 1000000 + round(total * 1e6 / rate), bytes(range(16)), trim, trim, trim, trim,
              json.dumps({"configuration": "test bow", "archer": "fixture"}).encode().ljust(512, b"\0"), 0)
    header = bytearray(MANIFEST.pack(*fields))
    struct.pack_into("<I", header, MANIFEST.size - 4, zlib.crc32(header[:-4]))
    return header, pages


def page_crc(page):
    used = struct.unpack_from("<I", page, 8)[0]
    struct.pack_into("<I", page, 12, 0)
    struct.pack_into("<I", page, 12, zlib.crc32(page[:used]))


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name)
        self.path = self.directory / "recording.bvr"

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, header=None, pages=None, **kwargs):
        if header is None: header, pages = fixture(**kwargs)
        self.path.write_bytes(header + b"".join(pages))
        return read_shot(self.path)

    def test_window_and_ring_wrap(self):
        shot = self.write()
        self.assertTrue(shot.metadata["valid"])
        self.assertEqual([len(a) for a in shot.arrays], [23040] * 4)
        self.assertAlmostEqual(shot.times[0][0], -1, places=5)
        self.assertAlmostEqual(shot.times[0][-1], 2 - 1 / 7680, places=5)

    def test_reject_manifest_crc(self):
        header, pages = fixture(); header[32] ^= 1
        with self.assertRaisesRegex(ValueError, "manifest"): self.write(header, pages)

    def test_no_trigger_keeps_full_recording(self):
        header, pages = fixture(total=153600)
        struct.pack_into("<II", header, 20, 10000, 3000)
        struct.pack_into("<Q", header, 56, 0)
        struct.pack_into("<I", header, MANIFEST.size - 4, zlib.crc32(header[:-4]))
        shot = self.write(header, pages)
        self.assertTrue(shot.metadata["valid"])
        self.assertFalse(shot.metadata["triggered"])
        self.assertEqual([len(a) for a in shot.arrays], [153600] * 4)
        self.assertAlmostEqual(shot.times[0][0], 0, places=5)
        self.assertAlmostEqual(shot.times[0][-1], 20 - 1 / 7680, places=5)
        export_shot(shot, self.directory, load_calibration(None))
        with np.load(next(self.directory.glob("*.npz")), allow_pickle=False) as data:
            self.assertAlmostEqual(float(data["capture_duration_s"]), 20, places=5)

    def test_early_pulse_preserves_available_history(self):
        header, pages = fixture(quality=0x20)
        struct.pack_into("<I", header, 20, 10000)
        struct.pack_into("<I", header, MANIFEST.size - 4, zlib.crc32(header[:-4]))
        shot = self.write(header, pages)
        self.assertEqual([len(a) for a in shot.arrays], [23040] * 4)
        self.assertAlmostEqual(shot.times[0][0], -1, places=5)
        self.assertFalse(shot.metadata["valid"])

    def test_reject_payload_and_header_corruption(self):
        for offset in (20, 70):
            header, pages = fixture(); pages[0][offset] ^= 1
            with self.assertRaisesRegex(ValueError, "CRC"): self.write(header, pages)

    def test_reject_truncation_and_extra_data(self):
        header, pages = fixture()
        for data in (header[:20], header + b"".join(pages)[:-10], header + b"".join(pages) + b"x"):
            self.path.write_bytes(data)
            with self.assertRaises(ValueError): read_shot(self.path)

    def test_reject_reordering(self):
        header, pages = fixture(); pages[0], pages[1] = pages[1], pages[0]
        with self.assertRaisesRegex(ValueError, "sequence"): self.write(header, pages)

    def test_reject_sample_gap(self):
        header, pages = fixture()
        # Second page starts with a later sensor-1 record. Alter its absolute index.
        struct.pack_into("<I", pages[1], PAGE.size + 8, 99999); page_crc(pages[1])
        with self.assertRaisesRegex(ValueError, "sequence"): self.write(header, pages)

    def test_quality_preserved_and_not_valid(self):
        shot = self.write(quality=0x81, saturation=2)
        self.assertFalse(shot.metadata["valid"])
        self.assertEqual(shot.metadata["quality_mask"], 0x81)
        self.assertEqual(len(shot.metadata["warnings"]), 2)

    def test_measured_timing_does_not_depend_on_trim_prediction(self):
        shot = self.write(trim=-37)
        self.assertTrue(shot.metadata["valid"])
        self.assertEqual(len(shot.arrays[0]), 23040)
        self.assertAlmostEqual(shot.metadata["sensor_estimated_sample_rates_hz"][0], 7680, places=2)

    def test_exports_work_with_existing_inspector(self):
        from inspect_raw import load_file
        shot = self.write(); export_shot(shot, self.directory, load_calibration(None))
        archive = next(self.directory.glob("*.npz"))
        with np.load(archive, allow_pickle=False) as data:
            np.testing.assert_array_equal(data["imu1_samples_lsb"], shot.arrays[0])
            self.assertIn("shot_metadata_json", data)
        self.assertIsNotNone(load_file(archive, None, 320, False, None))
        with self.assertRaises(FileExistsError): export_shot(shot, self.directory, load_calibration(None))

    def test_bad_orientation_rejected(self):
        calibration = load_calibration(None); calibration["rotations"][0] = np.diag([1, 1, -1]).tolist()
        path = self.directory / "calibration.json"; path.write_text(json.dumps(calibration))
        with self.assertRaisesRegex(ValueError, "rotation"): load_calibration(path)

    def test_analysis_and_repeat_statistics(self):
        shot = self.write()
        analyze(shot, load_calibration(None), self.directory / "analysis.png")
        compare([shot, shot], load_calibration(None), self.directory / "compare.png")
        report = json.loads((self.directory / "compare.json").read_text())
        self.assertEqual(report["repeat_statistics"][0]["valid_shots"], 2)
        self.assertEqual(report["repeat_statistics"][0]["std_peak_g"], 0)


if __name__ == "__main__": unittest.main()
