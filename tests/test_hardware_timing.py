"""Common pulses on independent clocks must retain their physical timing."""
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
import zlib

import numpy as np
from shot_tool import MANIFEST, PAGE, PAGE_BYTES, read_shot, export_shot, load_calibration

ROOT = Path(__file__).resolve().parents[1]
RATES = (7300, 7350, 7450, 7500)


def hardware_fixture(*, missing=None, bad_ticks=False, shifted_sensor=None, triggered=True, dense=False, tick_step=6, native=False):
    start = 1000000
    records = []
    for sensor, rate in enumerate(RATES):
        tick_offset = 24 + sensor * 2
        pulse = round(2 * rate - tick_offset / tick_step)
        if native:
            pulse = round((2000000 - tick_offset) * rate / 1e6)
        if shifted_sensor == sensor:
            pulse += round(.005 * rate)  # A real five-ms difference must survive.
        for index in range(0, rate * 3, 128):
            rows = min(128, rate * 3 - index)
            drain = start + round((index + rows - 1 + tick_offset / 6) * 1e6 / rate) + 200000 + sensor * 100000
            values = np.zeros((rows, 3), dtype='<i2'); values[:, 2] = 96
            if index <= pulse < index + rows:
                values[pulse - index, 0] = 6000
            if (dense or native) and missing != 'anchors':
                ticks = np.rint(np.arange(index, index + rows) * tick_step + tick_offset).astype(np.int64)
                if native:
                    ticks = np.rint(np.arange(index, index + rows) * 1e6 / rate + tick_offset).astype(np.int64)
                    ticks += np.where(np.arange(index, index + rows) % 2, 3, -3)
                if bad_ticks:
                    ticks[np.arange(index, index + rows) >= 512] += (100 if native else 60) * bad_ticks
                delta = np.r_[0, np.diff(ticks)]
                packed = bytearray()
                for xyz, dt in zip(values, delta):
                    packed.extend(xyz.tobytes()); packed.append(int(dt) & 255)
                frame = struct.pack('<QI4sBHI', drain, index, b'IM4S', sensor, rows, int(ticks[0])) + packed
            else:
                frame = struct.pack('<QI4sBH', drain, index, b'IM4D', sensor, rows) + values.tobytes()
            records.append((drain, frame))
            if triggered and sensor == 0 and index <= pulse < index + rows:
                records.append((drain, struct.pack('<QI4sBHI', drain, pulse, b'IM4T', sensor, 0, 50000)))
            if not dense and not native and missing != 'anchors':
                for anchor in range(index, index + rows):
                    if anchor % 32 == 0:
                        ticks = anchor * 6 + tick_offset + (60 * bad_ticks if bad_ticks and anchor >= 512 else 0)
                        records.append((drain, struct.pack('<QI4sBHI', drain, anchor, b'IM4H', sensor, 0, ticks)))
        if missing != 'clocks' and not native:
            for t in (.01, .25, .5, 1, 1.5, 2, 2.5, 3, 3.6):
                # One deliberately delayed calibration read should be discarded.
                span = 20000 if t == 1.5 else 12 + sensor * 2
                midpoint = start + round(t * 1e6) + (5000 if t == 1.5 else 0)
                ticks = round(t * rate * tick_step)
                drain = midpoint + span // 2 + 20
                records.append((drain, struct.pack('<QI4sBHIQI', drain, 0, b'IM4C', sensor, 0, ticks, midpoint, span)))
    records.sort(key=lambda r: r[0])  # Stable: trigger marker immediately follows its data.
    pages, payload, times = [], bytearray(), []

    def flush():
        nonlocal payload, times
        page = bytearray(b'\xff' * PAGE_BYTES)
        PAGE.pack_into(page, 0, b'BVP1', len(pages), PAGE.size + len(payload), 0, times[0], times[-1])
        page[PAGE.size:PAGE.size + len(payload)] = payload
        struct.pack_into('<I', page, 12, zlib.crc32(page[:PAGE.size + len(payload)]))
        pages.append(page); payload, times = bytearray(), []

    for drain, record in records:
        if PAGE.size + len(payload) + len(record) > PAGE_BYTES:
            flush()
        payload.extend(record); times.append(drain)
    if payload:
        flush()
    fields = (b'BVR1', 5 if native else 4 if dense else 3, 0, len(pages), 1775, 1000, 3000, 7680, 320, 15,
              0, 0, start, start + 2000000 if triggered else 0, records[-1][0] + 50,
              bytes(range(16)), 0, 0, 0, 0, b'{}'.ljust(512, b'\0'), 0)
    header = bytearray(MANIFEST.pack(*fields))
    struct.pack_into('<I', header, 604, zlib.crc32(header[:-4]))
    return header + b''.join(pages)


class HardwareTimingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / 'timed.bvr'

    def tearDown(self):
        self.tmp.cleanup()

    def read(self, **kwargs):
        self.path.write_bytes(hardware_fixture(**kwargs))
        return read_shot(self.path)

    def test_common_pulse_with_clock_drift_and_delayed_reads(self):
        shot = self.read()
        peaks = [t[np.argmax(v[:, 0])] for v, t in zip(shot.arrays, shot.times)]
        self.assertEqual(peaks[0], 0)
        self.assertLess(max(peaks) - min(peaks), 1 / min(RATES))
        np.testing.assert_allclose(shot.metadata['sensor_estimated_sample_rates_hz'], RATES, rtol=1e-8)
        for d in shot.metadata['timing_diagnostics']:
            self.assertEqual(d['clock_samples'], 8)
            self.assertLess(d['estimated_timing_uncertainty_us'], 40)

    def test_real_pulse_delay_is_preserved(self):
        shot = self.read(shifted_sensor=2)
        t = shot.times[2][np.argmax(shot.arrays[2][:, 0])]
        self.assertAlmostEqual(t, .005, delta=1 / min(RATES))

    def test_no_trigger_common_clock(self):
        shot = self.read(triggered=False)
        peaks = [t[np.argmax(v[:, 0])] for v, t in zip(shot.arrays, shot.times)]
        for t in peaks:
            self.assertAlmostEqual(t, 2, delta=1 / min(RATES))

    def test_missing_or_inconsistent_hardware_timing_rejected(self):
        for options in ({'missing': 'anchors'}, {'missing': 'clocks'}, {'bad_ticks': -1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.read(**options)

    def test_timestamp_gap_preserved_and_flagged(self):
        shot = self.read(bad_ticks=True)
        self.assertFalse(shot.metadata['valid'])
        self.assertTrue(any('discontinuities' in w for w in shot.metadata['warnings']))
        for d in shot.metadata['timing_diagnostics']:
            self.assertEqual(d['timestamp_discontinuities'], 1)
            self.assertEqual(d['fifo_fit_max_ticks'], 60)

    @unittest.skipUnless(shutil.which('node'), 'Node.js required')
    def test_browser_and_host_agree_and_export_common_times(self):
        shot = self.read(shifted_sensor=2, bad_ticks=True)
        output = self.path.with_suffix('.npz')
        script = r"""
const fs=require('fs');require(process.argv[1]);
(async()=>{
 const shot=BowVibFormat.readShot(new Uint8Array(fs.readFileSync(process.argv[2])));
 fs.writeFileSync(process.argv[3],Buffer.from(await BowVibFormat.shotArchive(shot).arrayBuffer()));
 console.log(JSON.stringify(shot.metadata.timing_diagnostics));
})().catch(e=>{console.error(e);process.exit(1)});
"""
        result = subprocess.run(['node', '-e', script, str(ROOT / 'main/web/format.js'), str(self.path), str(output)], check=True, capture_output=True, text=True)
        browser = json.loads(result.stdout)
        for a, b in zip(browser, shot.metadata['timing_diagnostics']):
            for key in a:
                self.assertAlmostEqual(a[key], b[key], places=6)
        with np.load(output) as archive:
            for i in range(4):
                np.testing.assert_allclose(archive[f'imu{i+1}_time_s'], shot.times[i], atol=1e-10)
                np.testing.assert_array_equal(archive[f'imu{i+1}_samples_lsb'], shot.arrays[i])
                if shot.sensor_ticks[i] is not None:
                    np.testing.assert_array_equal(archive[f'imu{i+1}_sensor_ticks'], shot.sensor_ticks[i])


class PerSampleTimingTests(HardwareTimingTests):
    def read(self, **kwargs):
        return super().read(dense=True, **kwargs)

    def test_gap_is_local_to_exact_sample_and_rates_stay_independent(self):
        shot = self.read(triggered=False, bad_ticks=True)
        for sensor, rate in enumerate(RATES):
            t = shot.times[sensor]
            idx = shot.indices[sensor]
            expected_ticks = idx.astype(float) * 6 + 24 + sensor * 2
            expected_ticks[idx >= 512] += 60
            np.testing.assert_array_equal(shot.sensor_ticks[sensor], expected_ticks)
            np.testing.assert_allclose(t, expected_ticks / (rate * 6), atol=1e-6, rtol=0)
            dt = np.diff(t)
            np.testing.assert_allclose(dt[np.arange(len(dt)) != 511], 1 / rate, atol=1e-8)
            self.assertAlmostEqual(dt[511], 11 / rate, delta=1e-8)
            self.assertEqual(shot.metadata['timing_diagnostics'][sensor]['fifo_timestamp_count'], rate * 3)
            self.assertLess(shot.metadata['timing_diagnostics'][sensor]['estimated_timing_uncertainty_us'], 40)

    def test_host_export_preserves_original_sensor_counters(self):
        shot = self.read(triggered=False)
        output = Path(self.tmp.name) / 'export'
        export_shot(shot, output, load_calibration(None), False)
        with np.load(output / ('shot_' + shot.metadata['id'] + '.npz')) as z:
            for i in range(4):
                np.testing.assert_array_equal(z[f'imu{i+1}_sensor_ticks'], shot.sensor_ticks[i])

    def test_quantized_timestamp_intervals_keep_independent_sample_times(self):
        shot = self.read(triggered=False, tick_step=5.75)
        self.assertTrue(all('window coverage' in w for w in shot.metadata['warnings']))
        for sensor, rate in enumerate(RATES):
            ticks = shot.sensor_ticks[sensor]
            self.assertEqual(set(np.diff(ticks)), {5, 6})
            # Calibration register reads are themselves quantized to one tick.
            np.testing.assert_allclose(shot.times[sensor], ticks / (rate * 5.75), atol=1 / (rate * 5.75), rtol=0)
            self.assertEqual(shot.metadata['timing_diagnostics'][sensor]['timestamp_discontinuities'], 0)


class DirectTimingTests(unittest.TestCase):
    def test_missing_and_backwards_direct_timestamps_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'direct.bvr'
            for options in ({'missing': 'anchors'}, {'bad_ticks': -2}):
                path.write_bytes(hardware_fixture(native=True, **options))
                with self.assertRaises(ValueError):
                    read_shot(path)

    def test_native_times_jitter_independent_rates_and_real_delay(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'direct.bvr'
            path.write_bytes(hardware_fixture(native=True, triggered=False, shifted_sensor=2))
            shot = read_shot(path)
            for sensor, rate in enumerate(RATES):
                idx = shot.indices[sensor]
                expected = np.rint(idx * 1e6 / rate + 24 + sensor * 2).astype(np.int64)
                expected += np.where(idx % 2, 3, -3)
                np.testing.assert_array_equal(shot.sensor_ticks[sensor], expected)
                np.testing.assert_allclose(shot.times[sensor], expected / 1e6, atol=1e-12, rtol=0)
                self.assertAlmostEqual(shot.metadata['sensor_estimated_sample_rates_hz'][sensor], rate, delta=.1)
            peaks = [t[np.argmax(v[:, 0])] for v, t in zip(shot.arrays, shot.times)]
            self.assertAlmostEqual(peaks[2] - peaks[0], .005, delta=1 / min(RATES))

    @unittest.skipUnless(shutil.which('node'), 'Node.js required')
    def test_direct_browser_host_export_and_gap_retention(self):
        script = r"""
const fs=require('fs');require(process.argv[1]);
(async()=>{
 const shot=BowVibFormat.readShot(new Uint8Array(fs.readFileSync(process.argv[2])));
 fs.writeFileSync(process.argv[3],Buffer.from(await BowVibFormat.shotArchive(shot).arrayBuffer()));
 console.log(JSON.stringify(shot.metadata.timing_diagnostics));
})().catch(e=>{console.error(e);process.exit(1)});
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'direct.bvr'
            for triggered in (False, True):
                path.write_bytes(hardware_fixture(native=True, triggered=triggered, bad_ticks=True, shifted_sensor=2))
                shot = read_shot(path)
                output = path.with_suffix('.npz')
                result = subprocess.run(['node', '-e', script, str(ROOT / 'main/web/format.js'), str(path), str(output)], check=True, capture_output=True, text=True)
                self.assertEqual(json.loads(result.stdout), shot.metadata['timing_diagnostics'])
                if triggered:
                    self.assertEqual(shot.times[0][np.argmax(shot.arrays[0][:, 0])], 0)
                else:
                    self.assertTrue(all(d['timestamp_discontinuities'] == 1 for d in shot.metadata['timing_diagnostics']))
                    self.assertTrue(all('exceed 200 us' in w or 'coverage' in w for w in shot.metadata['warnings']))
                    for sensor, rate in enumerate(RATES):
                        self.assertAlmostEqual(np.diff(shot.times[sensor])[511], 1 / rate + .000094, delta=1e-6)
                export_shot(shot, Path(directory) / 'host', load_calibration(None), True)
                with np.load(output) as browser, np.load(Path(directory) / 'host' / ('shot_' + shot.metadata['id'] + '.npz')) as host:
                    for i in range(4):
                        for key in ('samples_lsb', 'time_s', 'drdy_time_us'):
                            np.testing.assert_allclose(browser[f'imu{i+1}_{key}'], host[f'imu{i+1}_{key}'], atol=1e-12, rtol=0)
