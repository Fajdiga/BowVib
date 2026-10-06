"""Threshold sample metadata stays aligned through timing fits and exports."""
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
import zlib

import numpy as np

from shot_tool import MANIFEST, PAGE, PAGE_BYTES, export_shot, load_calibration, read_shot
from test_shot_tool import fixture


ROOT = Path(__file__).resolve().parents[1]
EVENT_INDEX, EVENT_SENSOR = 7684, 2
EVENT_INFO = dict(kind="threshold", sensor=EVENT_SENSOR, index=EVENT_INDEX, thresholdMg=50000)


def threshold_fixture(*, version=2, event=True, event_index=EVENT_INDEX, event_sensor=EVENT_SENSOR,
                      event_rows=0, threshold=50000, duplicate=False, triggered=True, isolated_event_page=False,
                      pre_ms=1000, post_ms=2000):
    """Use a drifting sensor clock and delayed FIFO callbacks, independent of trigger_us."""
    header, original_pages = fixture()
    records = []
    for page in original_pages:
        used = struct.unpack_from("<I", page, 8)[0]
        offset = PAGE.size
        while offset < used:
            _, index, _, sensor, rows = struct.unpack_from("<QI4sBH", page, offset)
            record = bytearray(page[offset:offset + 19 + rows * 6])
            drain = 1000000 + round((index + rows - 1) * 1e6 / 7800) + 6200 + sensor * 200
            struct.pack_into("<Q", record, 0, drain)
            if sensor == EVENT_SENSOR and index <= EVENT_INDEX < index + rows:
                # The stored trigger is the first acquired sample reaching 50 g.
                struct.pack_into("<hhh", record, 19 + (EVENT_INDEX - index) * 6, 5000, 0, 96)
                records.append((drain, record))
                if event:
                    marker = struct.pack("<QI4sBHI", drain, event_index, b"IM4T", event_sensor, event_rows, threshold)
                    records.extend([(drain, marker)] * (2 if duplicate else 1))
            else:
                records.append((drain, record))
            offset += 19 + rows * 6
    first = struct.unpack_from("<I", header, 8)[0]
    pages, payload, timestamps = [], bytearray(), []

    def flush():
        nonlocal payload, timestamps
        page = bytearray(b"\xff" * PAGE_BYTES)
        PAGE.pack_into(page, 0, b"BVP1", first + len(pages), PAGE.size + len(payload), 0, timestamps[0], timestamps[-1])
        page[PAGE.size:PAGE.size + len(payload)] = payload
        struct.pack_into("<I", page, 12, zlib.crc32(page[:PAGE.size + len(payload)]))
        pages.append(page)
        payload, timestamps = bytearray(), []

    for drain, record in records:
        is_marker = record[12:16] == b"IM4T"
        if isolated_event_page and is_marker and payload:
            flush()
        if PAGE.size + len(payload) + len(record) > PAGE_BYTES:
            flush()
        payload.extend(record)
        timestamps.append(drain)
        if isolated_event_page and is_marker:
            flush()
    if payload:
        flush()
    struct.pack_into("<I", header, 4, version)
    struct.pack_into("<I", header, 12, first + len(pages))
    struct.pack_into("<II", header, 20, pre_ms, post_ms)
    struct.pack_into("<Q", header, 56, 2001000 if triggered else 0)
    struct.pack_into("<Q", header, 64, records[-1][0] + 150)
    struct.pack_into("<I", header, MANIFEST.size - 4, zlib.crc32(header[:-4]))
    return header + b"".join(pages)


class TriggerMetadataTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name)
        self.path = self.directory / "shot.bvr"

    def tearDown(self):
        self.tmp.cleanup()

    def read(self, **kwargs):
        self.path.write_bytes(threshold_fixture(**kwargs))
        return read_shot(self.path)

    def test_threshold_sample_is_zero_after_clock_fit(self):
        shot = self.read()
        self.assertEqual(shot.metadata["trigger_info"], EVENT_INFO)
        self.assertGreater(abs(shot.metadata["trigger_alignment_shift_s"]), .005)
        self.assertAlmostEqual(shot.metadata["sensor_estimated_sample_rates_hz"][EVENT_SENSOR], 7800, places=2)
        i = np.flatnonzero(shot.indices[EVENT_SENSOR] == EVENT_INDEX).item()
        self.assertEqual(shot.times[EVENT_SENSOR][i], 0)
        self.assertLess(np.linalg.norm(shot.arrays[EVENT_SENSOR][i - 1]) * .010417, 50)
        self.assertGreaterEqual(np.linalg.norm(shot.arrays[EVENT_SENSOR][i]) * .010417, 50)
        for sensor in range(4):
            j = np.flatnonzero(shot.indices[sensor] == EVENT_INDEX).item()
            self.assertAlmostEqual(shot.times[sensor][j], (sensor - EVENT_SENSOR) * .0002, places=9)
        export_shot(shot, self.directory, load_calibration(None))
        with np.load(next(self.directory.glob("*.npz")), allow_pickle=False) as archive:
            self.assertEqual(json.loads(archive["shot_metadata_json"].item())["trigger_info"], EVENT_INFO)
            self.assertEqual(archive["imu3_time_s"][i], 0)

    def test_manual_and_legacy_keep_their_existing_origin(self):
        for version, triggered in ((1, True), (2, True), (2, False)):
            with self.subTest(version=version, triggered=triggered):
                shot = self.read(version=version, event=False, triggered=triggered)
                expected = {"kind": "manual"} if version == 2 and triggered else None
                self.assertEqual(shot.metadata["trigger_info"], expected)
                self.assertEqual(shot.metadata["trigger_alignment_shift_s"], 0)
                i = np.flatnonzero(shot.indices[EVENT_SENSOR] == EVENT_INDEX).item()
                expected_time = EVENT_INDEX / 7800 + .0066 - (1.001 if triggered else 0)
                self.assertAlmostEqual(shot.times[EVENT_SENSOR][i], expected_time, places=7)

    def test_malformed_event_metadata_is_rejected(self):
        for options in ({"event_index": 999999}, {"event_index": EVENT_INDEX - 256}, {"event_sensor": 4},
                        {"event_sensor": 1}, {"event_rows": 1}, {"threshold": 0}, {"threshold": 19},
                        {"threshold": 320001}, {"duplicate": True}, {"version": 1}, {"triggered": False}):
            with self.subTest(options=options), self.assertRaisesRegex(ValueError, "[Tt]hreshold trigger"):
                self.read(**options)

    def test_zero_pre_or_post_window_preserves_threshold_sample(self):
        for pre, post in ((0, 1000), (1000, 0)):
            with self.subTest(pre=pre, post=post):
                shot = self.read(pre_ms=pre, post_ms=post)
                i = np.flatnonzero(shot.indices[EVENT_SENSOR] == EVENT_INDEX).item()
                self.assertEqual(shot.times[EVENT_SENSOR][i], 0)
                for times in shot.times:
                    self.assertTrue(np.all(times >= -pre / 1000 - 1e-6))
                    self.assertTrue(np.all(times <= post / 1000 + 1e-6))
        with self.assertRaisesRegex(ValueError, "configuration"):
            self.read(pre_ms=0, post_ms=0)

    @unittest.skipUnless(shutil.which("node"), "Node.js required for browser decoder tests")
    def test_browser_and_host_agree_on_event_alignment_and_npz(self):
        script = r"""
const fs=require('fs'); require(process.argv[1]);
(async()=>{
  const shot=BowVibFormat.readShot(new Uint8Array(fs.readFileSync(process.argv[2])));
  fs.writeFileSync(process.argv[3],Buffer.from(await BowVibFormat.shotArchive(shot).arrayBuffer()));
  console.log(JSON.stringify({triggerInfo:shot.triggerInfo,counts:shot.counts,rates:shot.rates,shift:shot.metadata.trigger_alignment_shift_s}));
})().catch(e=>{console.error(e);process.exit(1)});
"""
        npz = self.directory / "browser.npz"
        for options in ({}, {"isolated_event_page": True}, {"pre_ms": 0}, {"post_ms": 0},
                        {"event": False}, {"event": False, "version": 1}, {"event": False, "triggered": False}):
            with self.subTest(options=options):
                shot = self.read(**options)
                result = subprocess.run(["node", "-e", script, str(ROOT / "main/web/format.js"), str(self.path), str(npz)], check=True, capture_output=True, text=True)
                browser = json.loads(result.stdout)
                self.assertEqual(browser["triggerInfo"], shot.metadata["trigger_info"])
                self.assertEqual(browser["counts"], shot.metadata["sensor_counts"])
                self.assertAlmostEqual(browser["shift"], shot.metadata["trigger_alignment_shift_s"], places=12)
                np.testing.assert_allclose(browser["rates"], shot.metadata["sensor_estimated_sample_rates_hz"])
                with np.load(npz, allow_pickle=False) as archive:
                    self.assertEqual(json.loads(archive["shot_metadata_json"].item())["trigger_info"], shot.metadata["trigger_info"])
                    for sensor in range(4):
                        np.testing.assert_array_equal(archive[f"imu{sensor + 1}_samples_lsb"], shot.arrays[sensor])
                        np.testing.assert_allclose(archive[f"imu{sensor + 1}_time_s"], shot.times[sensor], atol=1e-12)

    @unittest.skipUnless(shutil.which("node"), "Node.js required for browser decoder tests")
    def test_browser_rejects_malformed_event_metadata(self):
        script = "const fs=require('fs');require(process.argv[1]);BowVibFormat.readShot(new Uint8Array(fs.readFileSync(process.argv[2])));"
        for options in ({"event_index": 999999}, {"event_index": EVENT_INDEX - 256}, {"event_sensor": 4},
                        {"event_sensor": 1}, {"event_rows": 1}, {"threshold": 0}, {"threshold": 19},
                        {"threshold": 320001}, {"duplicate": True}, {"version": 1}, {"triggered": False}):
            with self.subTest(options=options):
                self.path.write_bytes(threshold_fixture(**options))
                result = subprocess.run(["node", "-e", script, str(ROOT / "main/web/format.js"), str(self.path)], capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("trigger", result.stderr)


if __name__ == "__main__":
    unittest.main()
