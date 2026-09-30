#!/usr/bin/env python3
"""Capture a ten-second raw IMU clip from a browser button.

Examples:
    python capture_button.py --wifi
    python capture_button.py --port COM92

The board remains idle until Start is pressed, then stops after ten seconds.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import io
import json
import socket
import sys
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import inspect_raw as protocol


CAPTURE_SECONDS = 10.0

PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BowVib capture</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e8edf3;font:16px system-ui,Segoe UI,sans-serif}
main{max-width:720px;margin:8vh auto;padding:24px}section{background:#171e28;border:1px solid #303b49;border-radius:12px;padding:24px}
h1{font-size:1.5rem;margin:0 0 16px}#status{color:#b6c5d6;line-height:1.5;white-space:pre-wrap;margin:16px 0}
button{width:100%;background:#16834b;color:white;border:0;border-radius:8px;padding:16px;font-size:1.1rem;font-weight:700;cursor:pointer;margin:8px 0}
button.secondary{background:#344354}button:disabled{opacity:.5;cursor:default}.hint{color:#9baabc;font-size:.9rem;line-height:1.5;margin-top:16px}
canvas{display:block;width:100%;height:auto;background:#171e28;border-radius:8px;margin-top:12px}#plotArea{margin-top:12px}
</style></head><body><main><section>
<h1>BowVib 10-second capture</h1>
<div id="status">Connecting to board...</div>
<button id="record" onclick="recordTenSeconds()" disabled>Start 10-second capture</button>
<button class="secondary" id="download" onclick="downloadCapture()" disabled>Save capture to this device</button>
<button class="secondary" id="plotButton" onclick="plotCapture()" disabled>Plot capture</button>
<button class="secondary" id="savePlot" onclick="savePlot()" disabled>Save plot as PNG</button>
<div id="plotArea" hidden><canvas id="chart" width="1200" height="1020"></canvas></div>
<div class="hint">The sensors start when you press the button. After ten seconds, acquisition stops and the raw XYZ data from each detected IMU is ready to download. The PC running this page must remain on during capture.</div>
</section></main>
<script>
async function recordTenSeconds(){const b=document.getElementById('record');b.disabled=true;try{const r=await fetch('/capture',{method:'POST'});const d=await r.json();if(!r.ok)throw Error(d.error||'Could not start capture');document.getElementById('status').textContent='Starting sensors...';document.getElementById('download').disabled=true}catch(e){alert(e.message);b.disabled=false}}
function downloadCapture(){window.location='/download'}
let lastPlot=null;
async function plotCapture(){try{const r=await fetch('/plot-data',{cache:'no-store'});if(!r.ok)throw Error('Could not load capture for plotting');lastPlot=await r.json();drawPlot(lastPlot);document.getElementById('plotArea').hidden=false;document.getElementById('savePlot').disabled=false}catch(e){alert(e.message)}}
function drawPlot(d){const c=document.getElementById('chart'),ctx=c.getContext('2d'),W=c.width,H=c.height;
 const colors=['#42a5f5','#ff9f43','#43c05b','#ff5252'],left=85,right=25,top=42,panel=292,gap=24,plotW=W-left-right;
 ctx.clearRect(0,0,W,H);ctx.font='18px system-ui';ctx.fillStyle='#e8edf3';ctx.textAlign='left';ctx.fillText(`BowVib capture ${d.timestamp} | ${d.duration.toFixed(3)} s`,left,25);
 const maxTime=Math.max(...d.series.map(s=>s.time.length?s.time[s.time.length-1]:0),d.duration);
 for(let axis=0;axis<3;axis++){const y0=top+axis*(panel+gap),y1=y0+panel-38;let lo=Infinity,hi=-Infinity;
  for(const s of d.series)for(const v of s.xyz){lo=Math.min(lo,v[axis]);hi=Math.max(hi,v[axis])}
  if(!Number.isFinite(lo)){lo=-1;hi=1}const pad=Math.max((hi-lo)*.08,.02);lo-=pad;hi+=pad;
  ctx.strokeStyle='#364252';ctx.fillStyle='#aebccd';ctx.lineWidth=1;ctx.textAlign='right';
  for(let k=0;k<=4;k++){const f=k/4,y=y1-f*(y1-y0),v=lo+f*(hi-lo);ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(W-right,y);ctx.stroke();ctx.fillText(v.toFixed(2),left-10,y+6)}
  ctx.textAlign='left';ctx.font='bold 19px system-ui';ctx.fillStyle='#e8edf3';ctx.fillText(`${'XYZ'[axis]} [g]`,12,y0+18);ctx.font='16px system-ui';
  for(let si=0;si<d.series.length;si++){const s=d.series[si];ctx.strokeStyle=colors[si%colors.length];ctx.lineWidth=1.4;ctx.beginPath();for(let i=0;i<s.time.length;i++){const x=left+s.time[i]/maxTime*plotW,y=y1-(s.xyz[i][axis]-lo)/(hi-lo)*(y1-y0);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)}ctx.stroke()}
  if(axis===0){let lx=left;ctx.font='15px system-ui';for(let si=0;si<d.series.length;si++){const s=d.series[si];ctx.fillStyle=colors[si%colors.length];ctx.fillText(`${s.label} ${s.rate.toFixed(1)}/${s.expected_rate.toFixed(1)} Hz`,lx,top-8);lx+=230}}
  if(axis===2){ctx.fillStyle='#aebccd';ctx.textAlign='center';ctx.fillText('time [s]',left+plotW/2,y1+32)}
 }
}
function savePlot(){const c=document.getElementById('chart');c.toBlob(blob=>{if(!blob)return;const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=`bowvib_capture_10s_${lastPlot.timestamp}_plot.png`;a.click();URL.revokeObjectURL(a.href)},'image/png')}
async function refresh(){try{const d=await(await fetch('/data',{cache:'no-store'})).json();const b=document.getElementById('record'),save=document.getElementById('download');
 const plot=document.getElementById('plotButton'),savePlotButton=document.getElementById('savePlot');
 if(d.recording){b.disabled=true;b.textContent=`Recording ${d.record_elapsed.toFixed(1)} / 10.0 s`;document.getElementById('status').textContent=`Receiving raw samples\nIMUs A-D: ${d.counts.map(x=>x.toLocaleString()).join(' / ')}`;save.disabled=true;plot.disabled=true;savePlotButton.disabled=true;document.getElementById('plotArea').hidden=true;lastPlot=null}
 else if(d.capture_building){b.disabled=true;b.textContent='Preparing file...';document.getElementById('status').textContent='Capture complete; preparing download.';save.disabled=true;plot.disabled=true;savePlotButton.disabled=true}
 else if(d.finished){b.disabled=false;b.textContent='Record another 10-second capture';document.getElementById('status').textContent=`Capture finished\nSamples A-D: ${d.counts.map(x=>x.toLocaleString()).join(' / ')}\nMeasured rates Hz: ${d.measured_rates_hz.map(x=>x.toFixed(1)).join(' / ')}\nTrim-predicted Hz: ${d.expected_rates_hz.map(x=>x.toFixed(1)).join(' / ')}\n${d.flags.join('\n')||'No overrun reported'}`;save.disabled=!d.capture_ready;plot.disabled=!d.capture_ready;savePlotButton.disabled=!lastPlot}
 else{b.disabled=false;b.textContent='Start 10-second capture';document.getElementById('status').textContent=`Ready\nDetected IMUs: ${d.sensor_ids.join(', ')||'none'}`;save.disabled=true;plot.disabled=true;savePlotButton.disabled=true}
 }catch(e){document.getElementById('status').textContent='Connection lost: '+e}}
setInterval(refresh,250);refresh();
</script></body></html>"""


@dataclass
class Source:
    kind: str
    connection: object
    rates: tuple[float, ...]
    present_ids: tuple[int, ...]
    stop_sent: threading.Event
    started: bool = False
    wifi_host: str | None = None
    wifi_port: int = 3333

    def disconnect(self) -> None:
        connection, self.connection = self.connection, None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass

    def reconnect(self) -> None:
        if self.kind != "wifi" or self.connection is not None:
            return
        if not self.wifi_host:
            raise RuntimeError("Wi-Fi reconnect has no board address")
        last_error: OSError | RuntimeError | None = None
        for attempt in range(3):
            connection = None
            try:
                connection = socket.create_connection(
                    (self.wifi_host, self.wifi_port), timeout=5.0
                )
                connection.settimeout(1.0)
                greeting = protocol.read_socket_line(connection, 8.0)
                if not greeting.startswith("BowVib"):
                    raise RuntimeError(f"unexpected board greeting: {greeting!r}")
                connection.sendall(b"STATUS\n")
                status = protocol.read_socket_line(connection, 4.0)
                if status.startswith("ERR "):
                    raise RuntimeError(status)
                mask = protocol.line_integer(status, "present_mask")
                expected_mask = sum(1 << (sensor_id - 1)
                                    for sensor_id in self.present_ids)
                if mask != expected_mask:
                    raise RuntimeError("sensor presence changed after reconnect")
                self.rates = parse_rates(status)
                self.connection = connection
                print("Wi-Fi stream reconnected to the board.")
                return
            except (OSError, RuntimeError) as exc:
                last_error = exc
                if connection is not None:
                    connection.close()
                if attempt < 2:
                    time.sleep(0.5 * (attempt + 1))
        raise RuntimeError(f"could not reconnect to board Wi-Fi stream: {last_error}")

    def read_exact(self, count: int, timeout: float = 15.0) -> bytes:
        if self.kind == "wifi":
            return protocol.read_socket_exact(self.connection, count, timeout)
        return protocol.read_exact(self.connection, count, timeout)

    def stop(self) -> None:
        if not self.started or self.stop_sent.is_set():
            return
        self.stop_sent.set()
        try:
            self.connection.sendall(b"STOP\n") if self.kind == "wifi" else (
                self.connection.write(b"STOP\n")
            )
            if self.kind == "usb":
                self.connection.flush()
        except (OSError, AttributeError):
            pass

    def start_capture(self) -> None:
        self.stop_sent.clear()
        if self.kind == "usb" and self.connection is None:
            raise RuntimeError("USB stream failed; restart the capture page to reconnect")
        if self.kind == "wifi":
            self.reconnect()
            try:
                self.connection.sendall(b"START\n")
                response = protocol.read_socket_line(self.connection, 4.0)
            except OSError:
                # A stale keep-alive socket is common after phone/PC Wi-Fi
                # changes or an ESP32 reset. Replace it once and retry START.
                self.disconnect()
                self.reconnect()
                self.connection.sendall(b"START\n")
                response = protocol.read_socket_line(self.connection, 4.0)
        else:
            response = protocol.send_and_wait(self.connection, "START", "CAPTURE START")
        if not response.startswith("CAPTURE START"):
            raise RuntimeError(f"unexpected start response: {response}")
        if protocol.line_integer(response, "present_mask") != sum(
                1 << (sensor_id - 1) for sensor_id in self.present_ids):
            raise RuntimeError("sensor presence changed before capture start")
        self.started = True

    def close(self) -> None:
        self.disconnect()


def parse_rates(status: str) -> tuple[float, ...]:
    trims: dict[str, int] = {}
    for token in status.split():
        if token.startswith("freq_fine="):
            for item in token.split("=", 1)[1].split(","):
                try:
                    label, raw = item.split(":", 1)
                    trims[label] = int(raw)
                except ValueError:
                    continue
    return tuple(
        protocol.HG_ODR_HZ * (1.0 + 0.0013 * trims.get(chr(65 + i), 0))
        if trims.get(chr(65 + i), 999) != 999 else float(protocol.HG_ODR_HZ)
        for i in range(protocol.SENSOR_COUNT)
    )


def open_source(args: argparse.Namespace) -> Source:
    if args.wifi:
        connection = socket.create_connection((args.wifi, args.wifi_port), timeout=10.0)
        connection.settimeout(1.0)
        greeting = protocol.read_socket_line(connection, 10.0)
        print(f"board> {greeting}")
        connection.sendall(b"STATUS\n")
        status = protocol.read_socket_line(connection, 4.0)
    else:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("USB mode needs pyserial: python -m pip install pyserial") from exc
        connection = serial.Serial(port=None, baudrate=115200, timeout=0.1)
        connection.dtr = False
        connection.rts = False
        connection.port = args.port
        connection.open()
        time.sleep(0.35)
        connection.reset_input_buffer()
        status = protocol.send_and_wait(connection, "STATUS", "STATUS")

    if status.startswith("ERR "):
        connection.close()
        raise RuntimeError(status)
    present_mask = protocol.line_integer(status, "present_mask")
    present_ids = tuple(i + 1 for i in range(protocol.SENSOR_COUNT)
                        if present_mask & (1 << i))
    if not present_ids:
        connection.close()
        raise RuntimeError("board reports no responding IMUs")

    print(f"Board ready: IMUs {', '.join(map(str, present_ids))}; capture starts on button press.")
    return Source("wifi" if args.wifi else "usb", connection,
                  parse_rates(status), present_ids, threading.Event(),
                  wifi_host=args.wifi, wifi_port=args.wifi_port)


class CaptureData:
    def __init__(self, source: Source) -> None:
        self.lock = threading.Lock()
        self.start_event = threading.Event()
        self.sensor_ids = source.present_ids
        self.expected_rates_hz = source.rates
        self.counts = [0] * protocol.SENSOR_COUNT
        self.error: str | None = None
        self.overrun_mask = 0
        self.finished = False
        self.recording = False
        self.record_started = 0.0
        self.record_chunks: list[list[np.ndarray]] = [
            [] for _ in range(protocol.SENSOR_COUNT)
        ]
        self.capture_archive: bytes | None = None
        self.capture_building = False
        self.capture_duration = 0.0
        self.capture_timestamp = ""

    def arm(self, now: float) -> bool:
        with self.lock:
            if self.recording or self.capture_building:
                return False
            self.record_chunks = [[] for _ in range(protocol.SENSOR_COUNT)]
            self.counts = [0] * protocol.SENSOR_COUNT
            self.capture_archive = None
            self.capture_duration = 0.0
            self.capture_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self.error = None
            self.overrun_mask = 0
            self.finished = False
            self.record_started = now
            self.recording = True
            return True

    def append(self, sensor: int, samples: np.ndarray) -> None:
        with self.lock:
            self.counts[sensor] += len(samples)
            if self.recording:
                self.record_chunks[sensor].append(samples.copy())

    def finish(self, source: Source, duration: float, footer_counts: tuple[int, ...],
               overrun_mask: int) -> None:
        with self.lock:
            chunks = [list(sensor_chunks) for sensor_chunks in self.record_chunks]
            received_counts = tuple(self.counts)
            self.capture_duration = duration
            self.recording = False
            self.finished = True
            self.overrun_mask = overrun_mask
            if received_counts != footer_counts:
                self.error = (f"frame/footer count mismatch: received={received_counts}, "
                              f"footer={footer_counts}")
                return
            self.capture_building = True
        threading.Thread(
            target=self._build_archive,
            args=(chunks, source, duration, overrun_mask),
            name="capture_archive", daemon=True,
        ).start()

    def fail(self, message: str) -> None:
        with self.lock:
            self.error = message
            self.recording = False
            self.finished = True

    def snapshot(self, now: float) -> dict[str, object]:
        with self.lock:
            return {
                "sensor_ids": self.sensor_ids,
                "counts": self.counts.copy(),
                "measured_rates_hz": [count / self.capture_duration
                                       if self.capture_duration > 0 else 0.0
                                       for count in self.counts],
                "expected_rates_hz": self.expected_rates_hz,
                "flags": ([f"FIFO overrun mask 0x{self.overrun_mask & 0x0F:X}"]
                           if self.finished and self.overrun_mask & 0x0F else []) +
                          (["Output transport overflow or send failure"]
                           if self.finished and self.overrun_mask & 0x80 else []) +
                          (["stream error: " + self.error] if self.error else []),
                "finished": self.finished,
                "recording": self.recording,
                "capture_building": self.capture_building,
                "record_elapsed": min(CAPTURE_SECONDS, max(0.0, now - self.record_started))
                                  if self.recording else self.capture_duration,
                "capture_ready": self.capture_archive is not None,
                "capture_timestamp": self.capture_timestamp,
            }

    def _build_archive(self, chunks: list[list[np.ndarray]], source: Source,
                       duration: float, overrun_mask: int) -> None:
        try:
            arrays = [np.concatenate(parts, axis=0) if parts else
                      np.empty((0, 3), dtype=np.int32) for parts in chunks]
            output = io.BytesIO()
            np.savez_compressed(
                output,
                **protocol.archive_payload(arrays, source.present_ids, duration, overrun_mask),
                sensor_expected_sample_rates_hz=np.asarray(source.rates, dtype=np.float64),
            )
            with self.lock:
                self.capture_archive = output.getvalue()
        except Exception as exc:
            self.fail(f"could not build capture archive: {exc}")
        finally:
            with self.lock:
                self.capture_building = False


def receive_stream(source: Source, data: CaptureData) -> None:
    while True:
        data.start_event.wait()
        data.start_event.clear()
        try:
            while True:
                if (not source.stop_sent.is_set() and
                        time.perf_counter() - data.record_started >= CAPTURE_SECONDS):
                    source.stop()
                frame = protocol.read_frame(source.read_exact)
                if frame[0] == b"IM4D":
                    if frame[1] + 1 not in source.present_ids:
                        raise RuntimeError("received data from an absent IMU")
                    data.append(frame[1], frame[2])
                else:
                    data.finish(source, frame[2], frame[1], frame[3])
                    source.started = False
                    break
        except Exception as exc:
            data.fail(str(exc))
            if not source.stop_sent.is_set():
                source.stop()
            source.disconnect()
            source.started = False
            if source.kind == "wifi":
                print(f"Wi-Fi stream disconnected; next capture will reconnect: {exc}")
                continue
            # Keep the reader available so a retry gets an explicit reconnect
            # error instead of starting a capture with no receiving thread.
            continue


def show_capture(source: Source, web_port: int, wifi_host: str | None) -> None:
    data = CaptureData(source)
    reader = threading.Thread(target=receive_stream, args=(source, data), daemon=True)
    reader.start()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args) -> None:
            pass

        def do_GET(self) -> None:
            if self.path == "/plot-data":
                with data.lock:
                    if data.capture_archive is None:
                        self.send_error(404, "No completed capture is ready to plot")
                        return
                    chunks = [list(parts) for parts in data.record_chunks]
                    ids = data.sensor_ids
                    duration = data.capture_duration
                    timestamp = data.capture_timestamp
                    expected_rates = data.expected_rates_hz
                max_points = 2500
                series = []
                for sensor_id in ids:
                    parts = chunks[sensor_id - 1]
                    values = (np.concatenate(parts, axis=0) if parts else
                              np.empty((0, 3), dtype=np.int32))
                    if len(values) == 0:
                        continue
                    rate = len(values) / duration if duration > 0 else 0.0
                    indices = np.linspace(0, len(values) - 1,
                                          min(len(values), max_points),
                                          dtype=np.int64)
                    points = (values[indices].astype(np.float64) *
                              (protocol.MILLIG_PER_LSB[320] / 1000.0))
                    series.append({
                        "sensor_id": sensor_id,
                        "label": f"Sensor {chr(64 + sensor_id)} (IMU {sensor_id})",
                        "rate": rate,
                        "expected_rate": expected_rates[sensor_id - 1],
                        "time": (indices / rate).round(6).tolist() if rate else [],
                        "xyz": points.round(5).tolist(),
                        "count": len(values),
                    })
                payload = json.dumps({
                    "timestamp": timestamp,
                    "duration": duration,
                    "series": series,
                }, separators=(",", ":")).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            if self.path == "/download":
                with data.lock:
                    archive = data.capture_archive
                    timestamp = data.capture_timestamp
                if archive is None:
                    self.send_error(404, "No completed capture is ready")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                filename = f"bowvib_capture_10s_{timestamp}.npz"
                self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(archive)))
                self.end_headers()
                self.wfile.write(archive)
                return
            if self.path == "/":
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/data":
                self.send_error(404)
                return
            payload = data.snapshot(time.perf_counter())
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:
            if self.path == "/capture":
                if not data.arm(time.perf_counter()):
                    body = json.dumps({"error": "A capture is active or acquisition has ended."}).encode()
                    self.send_response(409)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                try:
                    source.start_capture()
                except Exception as exc:
                    data.fail(str(exc))
                    body = json.dumps({"error": str(exc)}).encode()
                    self.send_response(503)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                data.start_event.set()
                body = json.dumps({"seconds": CAPTURE_SECONDS}).encode()
                self.send_response(202)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_error(404)

    server = ThreadingHTTPServer(("0.0.0.0", web_port), Handler)
    server.daemon_threads = True
    print(f"Open the 10-second capture page on this PC: http://127.0.0.1:{web_port}")
    for address in local_ipv4_addresses(source, wifi_host):
        print(f"Open from a phone on the same Wi-Fi: http://{address}:{web_port}")
    print("The board stays idle until Start is pressed; repeated clips are supported.")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        source.stop()
    finally:
        source.stop()
        server.server_close()
        reader.join(timeout=5.0)
        source.close()
    with data.lock:
        mask = data.overrun_mask
        finished = data.finished
    if finished and mask:
        print(f"Capture ended with status mask=0x{mask:02X}")
    elif finished:
        print("Capture ended with no FIFO or Wi-Fi queue overrun reported.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="USB Serial/JTAG port, e.g. COM92")
    source.add_argument("--wifi", nargs="?", const="192.168.4.1",
                        help="Wi-Fi host (default: 192.168.4.1)")
    parser.add_argument("--wifi-port", type=int, default=3333,
                        help="Wi-Fi TCP port (default: 3333)")
    parser.add_argument("--web-port", type=int, default=8765,
                        help="browser capture page port (default: 8765)")
    return parser


def local_ipv4_addresses(source: Source, wifi_host: str | None) -> list[str]:
    addresses: set[str] = set()
    if source.kind == "wifi" and wifi_host:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((wifi_host, 3333))
            address = probe.getsockname()[0]
            if not address.startswith("127."):
                addresses.add(address)
        except OSError:
            pass
        finally:
            probe.close()
    try:
        for result in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = result[4][0]
            if not address.startswith("127."):
                addresses.add(address)
    except OSError:
        pass
    return sorted(addresses)


def main() -> int:
    args = build_parser().parse_args()
    source = None
    try:
        source = open_source(args)
        show_capture(source, args.web_port, args.wifi)
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        if source:
            source.stop()
            source.close()
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
