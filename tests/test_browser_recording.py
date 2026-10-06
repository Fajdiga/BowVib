"""Cross-check the browser decoder and NPZ download against the host decoder."""
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
import zlib

import numpy as np

from test_shot_tool import fixture, MANIFEST
from test_trigger_metadata import threshold_fixture
from shot_tool import read_shot

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("node"), "Node.js required for browser decoder tests")
class BrowserRecordingTests(unittest.TestCase):
    def test_status_timeout_and_retry_recover_controls(self):
        script = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements={};
global.document={getElementById(id){return elements[id] ||= {value:'0',dataset:{},style:{},addEventListener(){}}}};
global.setInterval=()=>{};
const realSetTimeout=global.setTimeout;
global.setTimeout=(fn,ms)=>realSetTimeout(fn,ms===5000?10:ms);
function stalled(options){return new Promise((resolve,reject)=>options.signal.addEventListener('abort',()=>reject(Error('aborted'))))}
global.fetch=async(url,options)=>stalled(options);
vm.runInThisContext(fs.readFileSync(process.argv[1],'utf8'));
(async()=>{
  await new Promise(r=>realSetTimeout(r,30));
  assert(elements.status.textContent.includes('did not respond'));
  assert(elements.record.disabled);assert(!elements.retryConnection.disabled);
  assert(vm.runInThisContext('refreshing===false'));
  // A body that stalls after successful HTTP headers must also time out.
  global.fetch=async(url,options)=>({ok:true,json:()=>stalled(options)});
  await vm.runInThisContext('refresh()');
  assert(elements.status.textContent.includes('did not respond'));
  assert(vm.runInThisContext('refreshing===false'));
  global.fetch=async url=>({ok:true,json:async()=>url==='/status'?{present_mask:15}:{available:true,saved:false,running:false,state:'idle'}});
  elements.retryConnection.onclick();for(let i=0;i<25;i++)await Promise.resolve();
  assert(!elements.record.disabled,'recording enabled after successful retry');
  assert(elements.retryConnection.hidden);assert.strictEqual(elements.connectionBadge.dataset.state,'connected');
  // Capacity status continues beyond the former twenty-second deadline.
  global.fetch=async url=>({ok:true,json:async()=>url==='/status'?{present_mask:15}:{available:true,saved:false,running:true,state:'armed',ready:true,capacity_bytes:7270400,used_bytes:3635200,elapsed_ms:31000}});
  await vm.runInThisContext('refresh()');
  assert(elements.record.disabled);assert(!elements.trigger.disabled);
  assert.strictEqual(elements.progress.max,7270400);assert.strictEqual(elements.progress.value,3635200);
  assert(elements.status.textContent.includes('31.0 s'));assert(elements.status.textContent.includes('50% storage used'));
  assert(elements.captureHint.textContent.includes('until storage is full'));
})().catch(e=>{console.error(e);process.exit(1)});
"""
        subprocess.run(["node", "-e", script, str(ROOT / "main/web/capture.js")], check=True, capture_output=True, text=True)

    def test_startup_script_errors_show_reload_action(self):
        script = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[1],'utf8');
const bootstrap=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)][0][1];
const elements={};const listeners={};let watchdog;
global.document={getElementById(id){return elements[id] ||= {dataset:{},hidden:true}}};
global.window=global;global.addEventListener=(event,fn)=>{listeners[event]=fn};
global.setTimeout=fn=>{watchdog=fn;return 1};global.clearTimeout=()=>{};
vm.runInThisContext(bootstrap);
listeners.error({target:{tagName:'SCRIPT'}});
assert(elements.status.textContent.includes('could not load'));assert(!elements.reloadPage.hidden);
listeners.error({message:'SyntaxError: test error'});
assert(elements.status.textContent.includes('SyntaxError: test error'));
watchdog();assert(elements.status.textContent.includes('did not start'));
BowVibBoot.connected();assert(elements.reloadPage.hidden);
"""
        subprocess.run(["node", "-e", script, str(ROOT / "main/web/index.html")], check=True, capture_output=True, text=True)

    def test_capture_controls_and_zoom(self):
        script = r"""
const fs=require('fs'), vm=require('vm'), assert=require('assert');
require(process.argv[1]);
const html=fs.readFileSync(require('path').join(require('path').dirname(process.argv[2]),'index.html'),'utf8');
const htmlIds=new Set([...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]));
const elements={}, labels=[], context=new Proxy({fillText(text){labels.push(text)}}, {get:(o,k)=>o[k] ||= ()=>{}});
let chartWidth=1200;const frames=[];
global.requestAnimationFrame=fn=>{frames.push(fn);return frames.length};
const flush=()=>{while(frames.length)frames.shift()()};
global.document={getElementById(id){assert(htmlIds.has(id),'Missing HTML control: '+id);return elements[id] ||= {value:id==='plotMode'?'xyz':'0',checked:true,max:20,width:1200,height:1020,disabled:false,hidden:true,style:{},dataset:{},
  setAttribute(){},getContext:()=>context,addEventListener(name,fn){this[name]=fn},getBoundingClientRect:()=>({left:0,width:chartWidth}),setPointerCapture(){},hasPointerCapture:()=>false,reportValidity:()=>true}}};
global.setInterval=()=>{};
let lastCommand;
const bytes=fs.readFileSync(process.argv[3]);
const parsed=BowVibFormat.readShot(new Uint8Array(bytes));
global.fetch=async(url,options)=>{
  if(options?.method==='POST'){lastCommand={url,body:options.body};return {ok:true};}
  if(url==='/shot.bin')return {ok:true,arrayBuffer:async()=>bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.length)};
  return {ok:true,json:async()=>url==='/status'?{present_mask:15}:{available:true,saved:true,running:false,state:'saved',id:parsed.id}};
};
vm.runInThisContext(fs.readFileSync(process.argv[2],'utf8'));
(async()=>{
  // Allow the initial fetch and automatic plot loading to settle.
  for(let i=0;i<20;i++)await Promise.resolve();
  assert(!elements.plotArea.hidden);assert(!elements.downloadNpz.disabled);
  const state=()=>vm.runInThisContext('({start:viewStart,end:viewEnd,span:viewEnd-viewStart,full:fullEnd-fullStart})');
  const initial=state().span;
  assert(Math.abs(state().start+state().end)<1e-9,'auto-focus centers event');
  assert(initial<state().full,'auto-focus opens pulse detail');
  elements.zoomIn.onclick();assert(Math.abs(vm.runInThisContext('viewEnd-viewStart')-initial/2)<1e-9);
  elements.pan.value='1000';elements.pan.oninput();assert(Math.abs(vm.runInThisContext('viewEnd-fullEnd'))<1e-9);
  elements.resetZoom.onclick();assert(Math.abs(state().span-state().full)<1e-9);
  elements.focusEvent.onclick();assert(Math.abs(state().start+state().end)<1e-9);
  const event=(x,y,id=1,extra={})=>({clientX:x,clientY:y,pointerId:id,pointerType:'touch',button:0,preventDefault(){},...extra});
  // Pinch at a stationary midpoint zooms smoothly without a range selection on release.
  elements.chart.onpointerdown(event(400,150,1));elements.chart.onpointerdown(event(600,150,2));
  elements.chart.onpointermove(event(300,150,1));elements.chart.onpointermove(event(700,150,2));flush();
  assert(Math.abs(state().span-initial/2)<1e-9,'two fingers halve visible span');
  const pinched=state();elements.chart.onpointerup(event(700,150,2));
  assert.deepStrictEqual(state(),pinched,'lifting second finger does not select a range');
  elements.chart.onpointermove(event(330,150,1));flush();
  assert(Math.abs(state().span-pinched.span)<1e-9);assert(state().start<pinched.start,'remaining finger pans');
  elements.chart.onpointercancel(event(330,150,1));flush();
  assert(vm.runInThisContext('pointers.size===0 && gesture===null'),'cancel cleans up gesture');
  // Plain drag pans; Shift+mouse drag explicitly selects a range.
  elements.chart.onpointerdown(event(200,150,3,{pointerType:'mouse',shiftKey:true}));
  elements.chart.onpointermove(event(600,150,3));elements.chart.onpointerup(event(600,150,3));flush();
  assert(state().span<pinched.span);
  elements.focusEvent.onclick();
  const anchorBefore=vm.runInThisContext('pointerTime({clientX:600})');
  elements.chart.wheel({...event(600,150),deltaY:-20,deltaX:0,deltaMode:0,ctrlKey:true});flush();
  assert(state().span<initial,'trackpad pinch zooms');
  assert(Math.abs(vm.runInThisContext('pointerTime({clientX:600})')-anchorBefore)<1e-9,'zoom retains cursor anchor');
  for(let i=0;i<30;i++)elements.zoomIn.onclick();assert(Math.abs(state().span-.001)<1e-9);
  for(let i=0;i<30;i++)elements.zoomOut.onclick();assert(Math.abs(state().span-state().full)<1e-9);
  chartWidth=360;elements.plotMode.value='magnitude';elements.plotMode.onchange();
  assert.strictEqual(elements.chart.width,360);assert.strictEqual(elements.chart.style.height,'242px');
  assert(labels.includes('50 g threshold'),'magnitude shows the saved threshold');
  assert(elements.plotSummary.textContent.includes('sensor C'),'event description identifies triggering sensor');
  chartWidth=254;elements.plotMode.onchange();assert.strictEqual(elements.chart.width,254,'narrow phone drawing matches touch coordinates');
  for(let i=0;i<4;i++){elements['sensor'+i].checked=false;elements['sensor'+i].onchange()}
  elements.chart.onkeydown({key:'e',preventDefault(){}});assert(Math.abs(state().start+state().end)<1e-9);
  // No-trigger sessions open at full length with event navigation unavailable.
  vm.runInThisContext('loaded.triggered=false; viewStart=fullStart; viewEnd=fullEnd; drawPlot()');
  assert(elements.focusEvent.disabled);assert.strictEqual(state().span,state().full);
  elements.preSeconds.value='10';elements.postSeconds.value='3';elements.thresholdG.value='50';elements.shotMetadata.value='{}';
  elements.record.onclick();for(let i=0;i<20;i++)await Promise.resolve();
  assert(lastCommand.url.includes('pre_ms=10000'));assert(lastCommand.url.includes('post_ms=3000'));assert(lastCommand.url.includes('threshold_mg=50000'));
  global.fetch=async()=>{throw Error('Board disconnected')};await vm.runInThisContext('refresh()');
  assert(elements.record.disabled && elements.trigger.disabled && elements.delete.disabled);
  assert.strictEqual(elements.connectionBadge.dataset.state,'offline');
  assert.strictEqual(elements.status.textContent,'Board disconnected');
})().catch(e=>{console.error(e);process.exit(1)});
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shot.bvr"
            path.write_bytes(threshold_fixture())
            subprocess.run(["node", "-e", script, str(ROOT / "main/web/format.js"), str(ROOT / "main/web/capture.js"), str(path)], check=True, capture_output=True, text=True)

    def test_browser_matches_host_and_exports_timing(self):
        script = r"""
const fs=require('fs'); require(process.argv[1]);
(async()=>{
  const s=BowVibFormat.readShot(new Uint8Array(fs.readFileSync(process.argv[2])));
  const blob=BowVibFormat.shotArchive(s);
  fs.writeFileSync(process.argv[3],Buffer.from(await blob.arrayBuffer()));
  console.log(JSON.stringify({counts:s.counts,rates:s.rates,starts:s.series.map(x=>x.start),triggered:s.triggered,warnings:s.warnings}));
})().catch(e=>{console.error(e);process.exit(1)});
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shot.bvr"
            npz = Path(directory) / "shot.npz"
            for mode in ("triggered", "full", "early"):
                with self.subTest(mode=mode):
                    header, pages = fixture(quality=32 if mode == "early" else 0)
                    if mode != "triggered":
                        struct.pack_into("<II", header, 20, 10000, 3000 if mode == "full" else 2000)
                    if mode == "full":
                        struct.pack_into("<Q", header, 56, 0)
                    struct.pack_into("<I", header, MANIFEST.size - 4, zlib.crc32(header[:-4]))
                    path.write_bytes(header + b"".join(pages))
                    host = read_shot(path)
                    result = subprocess.run(["node", "-e", script, str(ROOT / "main/web/format.js"), str(path), str(npz)], check=True, capture_output=True, text=True)
                    browser = json.loads(result.stdout)
                    self.assertEqual(browser["counts"], host.metadata["sensor_counts"])
                    self.assertEqual(browser["triggered"], host.metadata["triggered"])
                    np.testing.assert_allclose(browser["rates"], host.metadata["sensor_estimated_sample_rates_hz"])
                    np.testing.assert_allclose(browser["starts"], [t[0] for t in host.times], atol=1e-9)
                    self.assertEqual(bool(browser["warnings"]), bool(host.metadata["warnings"]))
                    with np.load(npz, allow_pickle=False) as archive:
                        for i in range(4):
                            np.testing.assert_array_equal(archive[f"imu{i+1}_samples_lsb"], host.arrays[i])
                            np.testing.assert_allclose(archive[f"imu{i+1}_time_s"], host.times[i], atol=1e-9)
                        self.assertEqual(json.loads(archive["shot_metadata_json"].item())["triggered"], mode != "full")

    def test_browser_rejects_corrupt_crc_and_truncation(self):
        script = "const fs=require('fs');require(process.argv[1]);BowVibFormat.readShot(new Uint8Array(fs.readFileSync(process.argv[2])));"
        header, pages = fixture()
        original = header + b"".join(pages)
        damaged = bytearray(original); damaged[700] ^= 1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shot.bvr"
            for data in (original[:-10], damaged):
                path.write_bytes(data)
                result = subprocess.run(["node", "-e", script, str(ROOT / "main/web/format.js"), str(path)], capture_output=True)
                self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
