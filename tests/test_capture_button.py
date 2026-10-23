"""On-board capture integration checks."""
import io
import json
import shutil
import subprocess
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from capture_button import CaptureSettings, Recorder, Source, PAGE, build_parser, render_page
from test_hardware_timing import hardware_fixture
from shot_tool import read_shot


class FakeBoard:
    def __init__(self):
        self.status = dict(state='idle', available=True, saved=False, running=False,
                           ready=False, id='', error=0)
        self.calls = []
        self.raw = b''

    def request(self, action, settings=None):
        self.calls.append((action, settings))
        if action == 'status':
            return self.status.copy()
        if action == 'download':
            assert not self.status['running'], 'Download during acquisition'
            return self.raw
        if action == 'arm':
            self.status.update(state='armed', running=True, ready=True, saved=False)
        elif action == 'trigger':
            self.status.update(state='post', ready=False)
        elif action == 'cancel':
            self.status.update(state='idle', running=False, ready=False)
        elif action == 'delete':
            self.status.update(saved=False, state='idle')
        return b'{}'


class CaptureButtonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw = hardware_fixture(native=True)

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.board = FakeBoard()
        self.recorder = Recorder(self.board, self.directory.name)

    def actions(self):
        return [a for a, _ in self.board.calls]

    def complete(self):
        path = Path(self.directory.name) / 'fixture.bvr'
        path.write_bytes(self.raw)
        self.reference = read_shot(path)
        self.board.raw = self.raw
        self.board.status.update(saved=True, running=False, state='saved', ready=False,
                                 id=self.reference.metadata['id'])

    def test_deferred_download_preserves_original_timing(self):
        self.recorder.command('capture', {'mode': 'trigger'})
        self.recorder.poll()
        self.assertNotIn('download', self.actions())
        settings = next(s for a, s in self.board.calls if a == 'arm')
        self.assertEqual(settings.board_values(), (10000, 3000, 50000))
        self.complete()
        self.recorder.poll()
        with np.load(io.BytesIO(self.recorder.archive), allow_pickle=False) as archive:
            for i in range(4):
                np.testing.assert_array_equal(archive[f'imu{i+1}_samples_lsb'], self.reference.arrays[i])
                np.testing.assert_array_equal(archive[f'imu{i+1}_time_s'], self.reference.times[i])
                np.testing.assert_array_equal(archive[f'imu{i+1}_drdy_time_us'], self.reference.sensor_ticks[i])
        self.assertEqual(self.recorder.view['flags'], self.reference.metadata['warnings'])
        self.recorder.command('capture', {'mode': 'trigger'})
        self.assertLess(self.actions().index('download'), self.actions().index('delete'))
        self.assertTrue(self.recorder.raw_path.exists())
        self.assertTrue(self.recorder.npz_path.exists())

    def test_timed_manual_trigger_with_zero_postbuffer(self):
        with patch('capture_button.time.monotonic', return_value=100):
            self.recorder.command('capture', {'seconds': 2})
        self.assertEqual(self.recorder.settings.board_values(), (2000, 0, 0))
        with patch('capture_button.time.monotonic', return_value=101):
            self.recorder.poll()
        self.assertNotIn('trigger', self.actions())
        with patch('capture_button.time.monotonic', return_value=102):
            self.recorder.poll()
            self.recorder.poll()
        self.assertEqual(self.actions().count('trigger'), 1)

    def test_recovery_and_missing_backup_protection(self):
        self.complete()
        self.recorder.poll()
        self.assertTrue(self.recorder.view['capture_ready'])
        self.assertNotIn('delete', self.actions())
        self.recorder.npz_path.unlink()
        with self.assertRaisesRegex(RuntimeError, 'Save and verify'):
            self.recorder.command('capture')
        self.assertNotIn('delete', self.actions())

    def test_failed_download_never_deletes_recording(self):
        self.complete()
        self.board.raw = self.raw[:-30]
        with self.assertRaises(ValueError):
            self.recorder.command('capture', {'mode': 'trigger'})
        self.assertNotIn('delete', self.actions())
        self.assertNotIn('arm', self.actions())
        self.assertTrue(self.board.status['saved'])

    def test_shutdown_cancels_owned_capture(self):
        self.recorder.command('capture', {'mode': 'trigger'})
        self.recorder.shutdown()
        self.assertIn('cancel', self.actions())
        self.assertFalse(self.board.status['running'])

    def test_shutdown_does_not_cancel_other_client(self):
        self.board.status.update(running=True, state='armed', ready=True)
        self.recorder.poll()
        self.recorder.shutdown()
        self.assertNotIn('cancel', self.actions())

    def test_no_fifo_fallback(self):
        self.complete()
        self.board.raw = hardware_fixture(dense=True)
        with self.assertRaisesRegex(ValueError, 'not direct data-ready'):
            self.recorder.poll()
        self.assertNotIn('delete', self.actions())

    def test_settings_match_board_limits(self):
        for values in ({'seconds': 11}, {'pre_seconds': 11}, {'post_seconds': 4},
                       {'threshold_g': 321}, {'seconds': float('nan')},
                       {'mode': 'trigger', 'pre_seconds': 0, 'post_seconds': 0},
                       {'mode': 'trigger', 'threshold_g': .01}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                CaptureSettings.parse(values)

    def test_launch_parameters_prefill_browser_fields(self):
        args = build_parser().parse_args(['--wifi', '--pre', '5', '--threshold', '25',
                                         '--post', '2', '--seconds', '4'])
        settings = CaptureSettings.parse(dict(mode='trigger', pre_seconds=args.pre,
            threshold_g=args.threshold, post_seconds=args.post, seconds=args.seconds))
        page = render_page(settings)
        for name, value in [('pre_seconds', '5'), ('threshold_g', '25'),
                            ('post_seconds', '2'), ('seconds', '4')]:
            self.assertRegex(page, '<input id="' + name + '"[^>]*value="' + value + '"')
        self.assertEqual(build_parser().parse_args(['--port', 'COM92', '--threshold-g', '0']).threshold, 0)

    def test_usb_uses_shot_protocol(self):
        source = Source(port='COM92')
        with patch('shot_tool.usb_api', return_value=b'SHOT ARMED') as request:
            source.request('arm', CaptureSettings(mode='trigger'))
            self.assertTrue(request.call_args.args[1].startswith('SHOTARM 10000 3000 50000'))
        with patch('shot_tool.usb_api', return_value=b'SHOT state=armed available=1 saved=0 error=0 id='):
            status = source.request('status')
            self.assertTrue(status['running'])
            self.assertTrue(status['ready'])

    def test_wifi_uses_webpage_endpoints(self):
        source = Source(host='192.168.4.1')
        from unittest.mock import MagicMock
        reply = MagicMock()
        reply.__enter__.return_value.read.return_value = b'{}'
        with patch('capture_button.urllib.request.urlopen', return_value=reply) as request:
            source.request('arm', CaptureSettings(mode='trigger'))
            sent = request.call_args.args[0]
            self.assertEqual(sent.full_url, 'http://192.168.4.1/shot/arm?pre_ms=10000&post_ms=3000&threshold_mg=50000')
            self.assertEqual(sent.get_method(), 'POST')
            source.request('download')
            sent = request.call_args.args[0]
            self.assertEqual(sent.full_url, 'http://192.168.4.1/shot.bin')
            self.assertEqual(sent.get_method(), 'GET')

    @unittest.skipUnless(shutil.which('node'), 'Node.js required')
    def test_browser_controls(self):
        script = PAGE.split('<script>')[1].split('</script>')[0]
        harness = r"""
const assert=require('assert');const elements={};
global.document={getElementById(id){return elements[id] ||= {value:'10',reportValidity:()=>true}}};
global.setInterval=()=>{};global.alert=message=>{throw Error(message)};
let posted;let state={connected:true,can_start:true,can_trigger:false,recording:false,capture_ready:false,message:'Ready'};
global.fetch=async(url,options)=>{if(options?.method==='POST'){posted={url,body:options.body};return {ok:true}}return {ok:true,json:async()=>state}};
"""
        checks = r"""
(async()=>{
 for(let i=0;i<10;i++)await Promise.resolve();
 document.getElementById('threshold_g').value='50';document.getElementById('post_seconds').value='3';
 await startCapture('trigger');assert.strictEqual(posted.url,'/capture');assert.strictEqual(JSON.parse(posted.body).threshold_g,50);
 state={...state,recording:true,mode:'trigger',can_start:false,can_trigger:true};await refresh();
 assert(elements.record.disabled);assert(!elements.trigger.disabled);
 state={...state,connected:false,can_trigger:false,message:'Board offline'};await refresh();
 assert(elements.arm.disabled);assert(elements.trigger.disabled);assert.strictEqual(elements.status.textContent,'Board offline');
})().catch(e=>{console.error(e);process.exit(1)});
"""
        subprocess.run(['node', '-e', harness + script + checks], check=True, capture_output=True, text=True)


if __name__ == '__main__':
    unittest.main()
