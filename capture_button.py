#!/usr/bin/env python3
"""Browser controls for BowVib's on-board recorder (no live FIFO streaming).

python capture_button.py --wifi
python capture_button.py --port COM92
Completed recordings are verified and saved under --output-dir before reuse.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import urllib.request

import numpy as np
import shot_tool


@dataclass(frozen=True)
class CaptureSettings:
    mode: str = 'timed'
    seconds: float = 10.0
    threshold_g: float = 50.0
    pre_seconds: float = 10.0
    post_seconds: float = 3.0

    @classmethod
    def parse(cls, values):
        if not isinstance(values, dict):
            raise ValueError('Settings must be a JSON object')
        mode = values.get('mode', 'timed')
        if mode not in ('timed', 'trigger'):
            raise ValueError('Unknown capture mode')
        parsed = {}
        for name, default, low, high in (
            ('seconds', 10, .01, 10), ('threshold_g', 50, 0, 320),
            ('pre_seconds', 10, 0, 10), ('post_seconds', 3, 0, 3),
        ):
            value = float(values.get(name, default))
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{name} must be between {low} and {high}')
            parsed[name] = round(value * 1000) / 1000
        if mode == 'trigger':
            if not parsed['pre_seconds'] and not parsed['post_seconds']:
                raise ValueError('Choose a nonzero recording window')
            if 0 < parsed['threshold_g'] < .02:
                raise ValueError('Minimum automatic threshold is 0.02 g; use 0 for manual')
        return cls(mode=mode, **parsed)

    def board_values(self):
        if self.mode == 'timed':
            return round(self.seconds * 1000), 0, 0
        return (round(self.pre_seconds * 1000), round(self.post_seconds * 1000),
                round(self.threshold_g * 1000))


class Source:
    """Use exactly the shot protocol used by the board webpage / shot_tool."""
    def __init__(self, host=None, port=None):
        self.host, self.port = host, port

    def request(self, action, settings=None):
        pre, post, threshold = settings.board_values() if settings else (0, 0, 0)
        if self.port:
            command = {'status': 'SHOTSTATUS', 'download': 'SHOTGET',
                       'trigger': 'SHOTTRIGGER', 'cancel': 'SHOTCANCEL',
                       'delete': 'SHOTDELETE',
                       'arm': f'SHOTARM {pre} {post} {threshold} {{"source":"capture_button"}}'}[action]
            raw = shot_tool.usb_api(self.port, command, timeout=120 if action in ('arm', 'download') else 5)
        else:
            path = {'status': '/shot', 'download': '/shot.bin',
                    'arm': f'/shot/arm?pre_ms={pre}&post_ms={post}&threshold_mg={threshold}'}.get(action, '/shot/' + action)
            body = None if action in ('status', 'download') else b'{"source":"capture_button"}'
            request = urllib.request.Request(f'http://{self.host}{path}', data=body,
                                             headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=120 if action in ('arm', 'download') else 5) as response:
                raw = response.read()
        if action != 'status':
            return raw
        if not self.port:
            return json.loads(raw)
        values = dict(token.split('=', 1) for token in raw.decode().split() if '=' in token)
        state = values.get('state', 'error')
        return {'state': state, 'available': values.get('available') == '1',
                'saved': values.get('saved') == '1', 'id': values.get('id', ''),
                'error': int(values.get('error', '0')), 'corrupt': state == 'corrupt',
                'running': state in ('armed', 'post'), 'ready': state == 'armed'}


class Recorder:
    def __init__(self, source, output_dir):
        self.source = source
        self.output_dir = Path(output_dir)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.settings = CaptureSettings()
        self.started = 0.
        self.owned = False
        self.timed_trigger_sent = False
        self.archive = None
        self.shot = None
        self.saved_id = ''
        self.invalid_id = None
        self.validation_error = ''
        self.raw_path = None
        self.npz_path = None
        self.status = {}
        self.error = ''
        self.closing = False
        self.view = {'connected': False, 'recording': False, 'capture_building': False,
                     'capture_ready': False, 'counts': [0]*4, 'sensor_ids': [],
                     'finished': False, 'flags': [], 'record_elapsed': 0., 'seconds': 10.,
                     'mode': 'timed', 'triggered': False, 'can_start': False,
                     'can_trigger': False, 'message': 'Connecting to board...'}

    def publish(self, **changes):
        # Replace the whole mapping: HTTP readers never wait for a download lock.
        self.view = {**self.view, **changes}

    def store_recording(self, status):
        self.publish(capture_building=True, message='Capture complete; downloading and verifying on PC...')
        raw = self.source.request('download')
        self.output_dir.mkdir(parents=True, exist_ok=True)
        # Unique paths preserve earlier downloads even when validation fails.
        stem = 'bowvib_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        raw_path = self.output_dir / (stem + '.bvr')
        with raw_path.open('xb') as output:
            output.write(raw)
            output.flush()
            os.fsync(output.fileno())
        try:
            shot = shot_tool.read_shot(raw_path)
            if status.get('id') and shot.metadata['id'] != status['id']:
                raise ValueError('Board recording changed during download; kept the raw file for review')
            if shot.metadata.get('format_version') != 5:
                raise ValueError('This recording is not direct data-ready format v5. Update board firmware; no FIFO fallback is used.')
        except ValueError as exc:
            # Preserve the failed artifact without writing another copy on
            # every status poll. Restart the script to explicitly retry it.
            self.invalid_id = status.get('id', '')
            self.validation_error = f'{exc}. Raw file retained at {raw_path.resolve()}; restart to retry.'
            raise ValueError(self.validation_error) from exc
        buffer = io.BytesIO()
        np.savez_compressed(buffer, **shot_tool.archive_payload(shot, shot_tool.load_calibration(None)))
        npz_path = raw_path.with_suffix('.npz')
        with npz_path.open('xb') as output:
            output.write(buffer.getvalue())
            output.flush()
            os.fsync(output.fileno())
        self.shot, self.archive = shot, buffer.getvalue()
        self.saved_id, self.raw_path, self.npz_path = shot.metadata['id'], raw_path, npz_path
        self.owned = False
        self.publish(capture_ready=True, capture_building=False, finished=True,
                     counts=shot.metadata['sensor_counts'],
                     sensor_ids=[i+1 for i in range(4) if shot.metadata['present_mask'] & (1 << i)],
                     measured_rates_hz=shot.metadata['sensor_estimated_sample_rates_hz'],
                     flags=shot.metadata['warnings'],
                     message=f'Saved on PC: {npz_path.resolve()}')

    def poll(self):
        with self.lock:
            status = self.source.request('status')
            self.status = status
            active = bool(status.get('running'))
            if (active and self.owned and self.settings.mode == 'timed'
                    and not self.timed_trigger_sent and status.get('ready')
                    and time.monotonic() - self.started >= self.settings.seconds):
                self.source.request('trigger')
                self.timed_trigger_sent = True
            self.publish(connected=True, recording=active, triggered=status.get('state') == 'post',
                         mode=self.settings.mode, seconds=self.settings.seconds,
                         record_elapsed=max(0., time.monotonic()-self.started) if self.owned else 0.,
                         can_trigger=bool(status.get('ready')) and self.settings.mode != 'timed',
                         can_start=False, capture_building=False)
            if status.get('saved') and status.get('id') != self.saved_id and not active:
                if status.get('id', '') == self.invalid_id:
                    raise ValueError(self.validation_error)
                self.store_recording(status)
            if status.get('corrupt') or status.get('state') == 'corrupt':
                raise RuntimeError('Board recording is corrupt; it has not been deleted')
            if not status.get('available'):
                raise RuntimeError('On-board recording storage is unavailable; check firmware and partition table')
            if status.get('error') and not status.get('saved'):
                raise RuntimeError(f"Board recorder error {status['error']}")
            if not active:
                self.owned = False
            self.error = ''
            message = ('Board recording: waiting for pulse/manual trigger' if status.get('state') == 'armed'
                       else 'Board recording post-buffer' if active
                       else f'Saved on PC: {self.npz_path.resolve()}' if self.archive else 'Board ready')
            self.publish(message=message, can_start=not active and not self.closing,
                         can_trigger=bool(status.get('ready')) and (not self.owned or self.settings.mode == 'trigger'))

    def command(self, action, values=None):
        with self.lock:
            if self.closing:
                raise RuntimeError('Capture server is shutting down')
            if action == 'capture':
                settings = CaptureSettings.parse(values or {})
                self.poll()
                if not self.view['can_start']:
                    raise RuntimeError('Board is not ready for a new capture')
                if self.status.get('saved'):
                    if (self.status.get('id') != self.saved_id or not self.raw_path or
                            not self.raw_path.exists() or not self.npz_path.exists()):
                        raise RuntimeError('Save and verify the existing recording before replacing it')
                    self.source.request('delete')
                self.settings, self.started = settings, time.monotonic()
                self.owned, self.timed_trigger_sent = True, False
                self.source.request('arm', settings)
                self.archive, self.shot = None, None
                self.saved_id = ''
                self.publish(recording=True, capture_ready=False, finished=False, flags=[],
                             counts=[0]*4, can_start=False, mode=settings.mode,
                             seconds=settings.seconds, message='Recording on the board...')
            elif action in ('trigger', 'cancel'):
                self.source.request(action)
                if action == 'trigger':
                    self.timed_trigger_sent = True
            else:
                raise ValueError('Unknown command')

    def run(self):
        while not self.stop_event.is_set():
            try:
                self.poll()
            except Exception as exc:
                self.error = str(exc)
                self.publish(connected=False, can_start=False, can_trigger=False,
                             capture_building=False, message=str(exc))
            self.stop_event.wait(2 if self.error else .25)

    def shutdown(self):
        self.stop_event.set()
        with self.lock:
            self.closing = True
            if self.owned:
                status = self.source.request('status')
                if status.get('running'):
                    self.source.request('cancel')
                deadline = time.monotonic() + 20
                while status.get('running') and time.monotonic() < deadline:
                    time.sleep(.2)
                    status = self.source.request('status')
                if status.get('running'):
                    raise TimeoutError('Board stop was not confirmed; recording remains on board')
                if status.get('saved') and status.get('id') != self.saved_id:
                    self.store_recording(status)
            self.owned = False

    def plot_data(self):
        with self.lock:
            if self.shot is None:
                raise ValueError('No completed capture is ready')
            series = []
            for i, (samples, times) in enumerate(zip(self.shot.arrays, self.shot.times)):
                if not len(times):
                    continue
                indices = np.linspace(0, len(times)-1, min(len(times), 2500), dtype=np.int64)
                rate = self.shot.metadata['sensor_estimated_sample_rates_hz'][i]
                series.append({'label': f'Sensor {chr(65+i)}', 'rate': rate, 'expected_rate': 7680,
                               'time': times[indices].tolist(),
                               'xyz': (samples[indices].astype(float)*.010417).tolist()})
            return {'timestamp': self.saved_id, 'duration': max(t[-1] for t in self.shot.times if len(t)) - min(t[0] for t in self.shot.times if len(t)), 'series': series}


def show_capture(source, web_port, output_dir):
    recorder = Recorder(source, output_dir)
    worker = threading.Thread(target=recorder.run, daemon=True)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def respond(self, status, body, content_type='application/json', filename=None):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            if filename:
                self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                if self.path == '/':
                    self.respond(200, PAGE.encode(), 'text/html; charset=utf-8')
                elif self.path == '/data':
                    self.respond(200, json.dumps(recorder.view).encode())
                elif self.path == '/plot-data':
                    self.respond(200, json.dumps(recorder.plot_data()).encode())
                elif self.path == '/download' and recorder.archive:
                    self.respond(200, recorder.archive, 'application/octet-stream', f'bowvib_{recorder.saved_id}.npz')
                else:
                    self.send_error(404)
            except (ValueError, OSError) as exc:
                self.send_error(400, str(exc))

        def do_POST(self):
            if self.path not in ('/capture', '/trigger', '/cancel'):
                self.send_error(404)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 4096:
                    raise ValueError('Settings request too large')
                values = json.loads(self.rfile.read(length)) if length else {}
                recorder.command(self.path[1:], values)
                self.respond(202, b'{}')
            except Exception as exc:
                self.respond(400, json.dumps({'error': str(exc)}).encode())

    server = ThreadingHTTPServer(('0.0.0.0', web_port), Handler)
    server.daemon_threads = True
    worker.start()
    print(f'Open http://127.0.0.1:{web_port} (on-board capture; download after completion)')
    print(f'Recordings are saved in {Path(output_dir).resolve()}')
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        print('\nStopping owned capture and preserving completed recording...')
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            try:
                recorder.shutdown()
            except Exception as exc:
                print(f'Could not confirm clean shutdown: {exc}. Reconnect to recover any saved board recording.')
            worker.join(timeout=6)
            server.server_close()
        finally:
            signal.signal(signal.SIGINT, previous)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--wifi', nargs='?', const='192.168.4.1', help='Board HTTP host, optionally host:port')
    group.add_argument('--port', help='USB serial port, e.g. COM92')
    parser.add_argument('--web-port', type=int, default=8765)
    parser.add_argument('--output-dir', type=Path, default=Path('captures'))
    return parser


def main():
    args = build_parser().parse_args()
    show_capture(Source(args.wifi, args.port), args.web_port, args.output_dir)
    return 0


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BowVib capture</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e8edf3;font:16px system-ui,Segoe UI,sans-serif}
main{max-width:720px;margin:8vh auto;padding:24px}section{background:#171e28;border:1px solid #303b49;border-radius:12px;padding:24px}
h1{font-size:1.5rem;margin:0 0 16px}#status{color:#b6c5d6;line-height:1.5;white-space:pre-wrap;margin:16px 0}
button{width:100%;background:#16834b;color:white;border:0;border-radius:8px;padding:16px;font-size:1.1rem;font-weight:700;cursor:pointer;margin:8px 0}
button.secondary{background:#344354}button:disabled{opacity:.5;cursor:default}.hint{color:#9baabc;font-size:.9rem;line-height:1.5;margin-top:16px}
.settings{display:flex;gap:12px;flex-wrap:wrap}.settings label{flex:1;min-width:120px}input{display:block;width:100%;margin:6px 0 10px;padding:10px;background:#10151c;color:#e8edf3;border:1px solid #607086;border-radius:6px;font-size:1.1rem}
canvas{display:block;width:100%;height:auto;background:#171e28;border-radius:8px;margin-top:12px}#plotArea{margin-top:12px}
</style></head><body><main><section>
<h1>BowVib capture</h1>
<div id="status">Connecting to board...</div>
<div class="settings"><label>Interval [s]<input id="seconds" type="number" min="0.01" max="10" step="0.001" value="10"></label></div>
<button id="record" onclick="startCapture('timed')" disabled>Start capture</button>
<div class="settings">
<label>Trigger [g]<input id="threshold_g" type="number" min="0" max="320" step="any" value="50"></label>
<label>Pre-buffer [s]<input id="pre_seconds" type="number" min="0" max="10" step="0.001" value="10"></label>
<label>Post-buffer [s]<input id="post_seconds" type="number" min="0" max="3" step="0.001" value="3"></label>
</div>
<button id="arm" onclick="startCapture('trigger')" disabled>Start trigger capture</button>
<button id="trigger" onclick="command('/trigger')" disabled>Trigger now</button>
<button class="secondary" id="cancel" onclick="command('/cancel')" disabled>Cancel capture</button>
<button class="secondary" id="download" onclick="downloadCapture()" disabled>Save capture to this device</button>
<button class="secondary" id="plotButton" onclick="plotCapture()" disabled>Plot capture</button>
<button class="secondary" id="savePlot" onclick="savePlot()" disabled>Save plot as PNG</button>
<div id="plotArea" hidden><canvas id="chart" width="1200" height="1020"></canvas></div>
<div class="hint">The board captures directly from the sensors and saves locally, just like its built-in webpage. Data transfers to this PC only after recording finishes, preserving original sample timestamps. Trigger 0 means manual only. Timed capture keeps the selected interval before a PC-issued trigger; keep the PC connected for that mode. Pre-buffer is up to 10 s and post-buffer up to 3 s. Without a trigger, the board saves when storage is full. Completed .bvr and .npz files are automatically saved on this PC before a new capture replaces the board recording.</div>
</section></main>
<script>
let pending=false;
async function command(path,body){pending=true;try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});if(!r.ok)throw Error(await r.text());lastPlot=null;document.getElementById('plotArea').hidden=true}catch(e){alert(e.message)}finally{pending=false;await refresh()}}
async function startCapture(mode){const body={mode};for(const id of ['seconds','threshold_g','pre_seconds','post_seconds']){const input=document.getElementById(id);if(!input.value||!input.reportValidity())return;body[id]=Number(input.value)}await command('/capture',body)}
function downloadCapture(){window.location='/download'}
let lastPlot=null;
async function plotCapture(){try{const r=await fetch('/plot-data',{cache:'no-store'});if(!r.ok)throw Error('Could not load capture for plotting');lastPlot=await r.json();drawPlot(lastPlot);document.getElementById('plotArea').hidden=false;document.getElementById('savePlot').disabled=false}catch(e){alert(e.message)}}
function drawPlot(d){const c=document.getElementById('chart'),ctx=c.getContext('2d'),W=c.width,H=c.height;
 const colors=['#42a5f5','#ff9f43','#43c05b','#ff5252'],left=85,right=25,top=42,panel=292,gap=24,plotW=W-left-right;
 ctx.clearRect(0,0,W,H);ctx.font='18px system-ui';ctx.fillStyle='#e8edf3';ctx.textAlign='left';ctx.fillText(`BowVib capture ${d.timestamp} | ${d.duration.toFixed(3)} s`,left,25);
 const minTime=Math.min(0,...d.series.map(s=>s.time[0]||0)),maxTime=Math.max(0,...d.series.map(s=>s.time.at(-1)||0)),span=Math.max(.001,maxTime-minTime);
 for(let axis=0;axis<3;axis++){const y0=top+axis*(panel+gap),y1=y0+panel-38;let lo=Infinity,hi=-Infinity;
  for(const s of d.series)for(const v of s.xyz){lo=Math.min(lo,v[axis]);hi=Math.max(hi,v[axis])}
  if(!Number.isFinite(lo)){lo=-1;hi=1}const pad=Math.max((hi-lo)*.08,.02);lo-=pad;hi+=pad;
  ctx.strokeStyle='#364252';ctx.fillStyle='#aebccd';ctx.lineWidth=1;ctx.textAlign='right';
  for(let k=0;k<=4;k++){const f=k/4,y=y1-f*(y1-y0),v=lo+f*(hi-lo);ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(W-right,y);ctx.stroke();ctx.fillText(v.toFixed(2),left-10,y+6)}
  ctx.textAlign='left';ctx.font='bold 19px system-ui';ctx.fillStyle='#e8edf3';ctx.fillText(`${'XYZ'[axis]} [g]`,12,y0+18);ctx.font='16px system-ui';
  for(let si=0;si<d.series.length;si++){const s=d.series[si];ctx.strokeStyle=colors[si%colors.length];ctx.lineWidth=1.4;ctx.beginPath();for(let i=0;i<s.time.length;i++){const x=left+(s.time[i]-minTime)/span*plotW,y=y1-(s.xyz[i][axis]-lo)/(hi-lo)*(y1-y0);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)}ctx.stroke()}
  if(axis===0){let lx=left;ctx.font='15px system-ui';for(let si=0;si<d.series.length;si++){const s=d.series[si];ctx.fillStyle=colors[si%colors.length];ctx.fillText(`${s.label} ${s.rate.toFixed(1)}/${s.expected_rate.toFixed(1)} Hz`,lx,top-8);lx+=230}}
  if(axis===2){ctx.fillStyle='#aebccd';ctx.textAlign='center';ctx.fillText(`time relative to trigger [s]: ${minTime.toFixed(3)} to ${maxTime.toFixed(3)}`,left+plotW/2,y1+32)}
 }
}
function savePlot(){const c=document.getElementById('chart');c.toBlob(blob=>{if(!blob)return;const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=`bowvib_capture_${lastPlot.timestamp}_plot.png`;a.click();URL.revokeObjectURL(a.href)},'image/png')}
let refreshing=false;
async function refresh(){if(refreshing)return;refreshing=true;try{
 const response=await fetch('/data',{cache:'no-store',signal:AbortSignal.timeout(5000)});if(!response.ok)throw Error('Status request failed');const d=await response.json();
 const busy=d.recording||d.capture_building||pending;
 for(const id of ['record','arm','seconds','threshold_g','pre_seconds','post_seconds'])document.getElementById(id).disabled=busy||!d.connected||!d.can_start;
 document.getElementById('trigger').disabled=!d.connected||!d.can_trigger||pending;
 document.getElementById('cancel').disabled=!d.connected||!d.recording||pending;
 document.getElementById('download').disabled=busy||!d.capture_ready;
 document.getElementById('plotButton').disabled=busy||!d.capture_ready;
 document.getElementById('savePlot').disabled=busy||!lastPlot;
 let status=d.message;
 if(d.recording){if(d.mode==='timed')status+=`\nElapsed ${d.record_elapsed.toFixed(1)} / ${d.seconds} s`;lastPlot=null;document.getElementById('plotArea').hidden=true}
 else if(d.capture_ready)status+=`\nSaved samples A-D: ${d.counts.join(' / ')}\nMeasured Hz: ${d.measured_rates_hz.map(x=>x.toFixed(1)).join(' / ')}\n${d.flags.join('\n')||'No data quality warnings reported'}`;
 document.getElementById('status').textContent=status;
 }catch(e){document.getElementById('status').textContent='Connection lost: '+e;for(const id of ['record','arm','trigger','cancel','download','plotButton','savePlot'])document.getElementById(id).disabled=true}finally{refreshing=false}}
setInterval(refresh,250);refresh();
</script></body></html>"""



if __name__ == "__main__":
    raise SystemExit(main())
