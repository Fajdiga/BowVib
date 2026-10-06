"use strict";
const $ = id => document.getElementById(id);
const format = globalThis.BowVibFormat;
let shot, board, busy = false, refreshing = false, loading = false, loaded, rawBlob, archiveBlob;
let loadError = "", commandError = "", connectionError = "", viewStart = 0, viewEnd = 1, fullStart = 0, fullEnd = 1;
let postStarted = 0, requestedPost = 3, connected = false;
let gesture = null, selection = null, plotFrame = 0;
const pointers = new Map();
const colors = ["#60a5fa", "#fb923c", "#4ade80", "#f472b6"];

// Cover both response headers and body: a lost Wi-Fi connection can leave
// either pending indefinitely, otherwise locking out every subsequent poll.
async function readBoard(path, type = "json", timeoutMs = 5000) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(path, {cache: "no-store", signal: controller.signal});
    if (!response.ok) throw Error("Board request " + path + " failed (HTTP " + response.status + "). Reload the page after updating firmware.");
    return await response[type]();
  } catch (err) {
    if (controller.signal.aborted) throw Error("BowVib did not respond. Connect to BowVib-IMU Wi-Fi, then retry. Retrying automatically.");
    throw err;
  } finally { clearTimeout(timer); }
}

function render() {
  const active = shot?.running;
  $("record").disabled = !connected || busy || loading || !shot?.available || active || shot.saved || shot.corrupt || !board?.present_mask;
  $("record").textContent = busy ? "Preparing recording..." : active ? "Recording..." : "Start recording";
  $("trigger").disabled = !connected || busy || !shot?.ready;
  $("cancel").disabled = !connected || busy || !["armed", "post"].includes(shot?.state);
  $("delete").disabled = !connected || busy || loading || active || !(shot?.saved || shot?.corrupt);
  $("download").disabled = busy || active || !shot?.saved;
  $("downloadNpz").disabled = !archiveBlob;
  $("plot").disabled = busy || active || !shot?.saved || loading;
  for (const id of ["thresholdG", "preSeconds", "postSeconds", "shotMetadata"]) $(id).disabled = busy || active;
  $("connectionBadge").textContent = connected ? (active ? "Recording" : "Board connected") : "Board offline";
  $("connectionBadge").dataset.state = connected ? (active ? "recording" : "connected") : "offline";
  $("retryConnection").hidden = connected;
  $("retryConnection").disabled = refreshing || busy;
  if (busy) $("status").textContent = "Applying command; preparing flash before a new recording...";
  else if (commandError) $("status").textContent = commandError;
  else if (connectionError) $("status").textContent = connectionError;
  else if (!shot || !board) $("status").textContent = "Connecting to BowVib...";
  else if (shot.state === "armed") {
    const elapsed = (shot.elapsed_ms ?? (shot.arm_limit_ms - shot.remaining_ms)) / 1000;
    const capacity = shot.capacity_bytes ?? shot.arm_limit_ms;
    const used = shot.used_bytes ?? (shot.arm_limit_ms - shot.remaining_ms);
    $("progress").max = capacity; $("progress").value = used;
    $("status").textContent = 'Recording · ' + elapsed.toFixed(1) + ' s · ' + (100 * used / capacity).toFixed(0) + '% storage used\nWaiting for a pulse. No pulse → save when storage is full.';
  } else if (shot.state === "post") {
    $("progress").max = requestedPost;
    $("progress").value = Math.min(requestedPost, (performance.now() - postStarted) / 1000);
    $("status").textContent = "Pulse detected · recording the tail...\nSaving the window around the pulse.";
  } else if (shot.saved) {
    $("progress").value = $("progress").max;
    $("status").textContent = loading ? "Recording saved on board · loading plot..." :
      loaded ? (loaded.triggered ? "Pulse window" : "No pulse · full recording") + ' saved · ' + loaded.duration.toFixed(3) + ' s\nReady to plot or save. Delete from board to record again.' : "Recording saved on board. Ready to save or plot.";
  } else if (shot.corrupt) $("status").textContent = "Stored recording is damaged. Delete it to record again.";
  else if (shot.error) $("status").textContent = 'Recorder error ' + shot.error + '. Please try again.';
  else if (!shot.available) $("status").textContent = "Recording storage unavailable. Flash the updated firmware and partition table.";
  else if (active) $("status").textContent = "Board is recording from another connection.";
  else {
    $("progress").value = 0;
    $("status").textContent = 'Ready · sensors ' + ([0,1,2,3].filter(i => board.present_mask & (1 << i)).map(i => "ABCD"[i]).join(", ") || "none") + '\nPress Start recording to begin.';
  }
  $("result").textContent = loadError || (loaded ? 'Samples A–D: ' + loaded.counts.join(" / ") + '\nEstimated Hz: ' + loaded.rates.map((r,i) => loaded.counts[i] ? r.toFixed(1) : "—").join(" / ") + (loaded.warnings.length ? "\nReview: " + loaded.warnings.join("; ") : "\nNo data quality warning reported.") : "");
}
async function refresh() {
  if (busy || refreshing) return;
  refreshing = true;
  try {
    const [next, nextBoard] = await Promise.all([readBoard("/shot"), readBoard("/status")]);
    if (!next || typeof next.available !== "boolean" || typeof next.saved !== "boolean" ||
        !nextBoard || !Number.isInteger(nextBoard.present_mask)) throw Error("Unexpected board status. Reload the page after updating firmware.");
    board = nextBoard; connected = true; connectionError = "";
    globalThis.BowVibBoot?.connected();
    if (next.state === "post" && shot?.state !== "post") postStarted = performance.now();
    shot = next; render();
    if (shot.saved && !shot.running && !loading && !loadError && loaded?.id !== shot.id) await loadRecording();
  } catch (err) { connected = false; connectionError = err.message; render(); }
  finally { refreshing = false; $("retryConnection").disabled = busy; }
}
$("retryConnection").onclick = () => { connectionError = ""; refresh(); };
async function command(name, settings) {
  if (busy) return;
  busy = true; commandError = ""; render();
  try {
    const query = name === "arm" ? "?" + new URLSearchParams(settings.values) : "";
    const response = await fetch('/shot/' + name + query, {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(settings?.metadata || {})});
    if (!response.ok) throw Error(await response.text());
    if (name === "arm" || name === "delete") {
      loaded = rawBlob = archiveBlob = null; loadError = "";
      pointers.clear(); gesture = selection = null;
      $("plotArea").hidden = true; $("savePlot").disabled = true;
    }
  } catch (err) { commandError = err.message; }
  finally { busy = false; await refresh(); render(); }
}
function updateHint() {
  const threshold = Number($("thresholdG").value);
  $("captureHint").textContent = (threshold ? 'The first sample reaching ' + threshold + ' g triggers the recording window.' : 'Manual mode: press Trigger now during recording.') + ' Keep up to ' + $("preSeconds").value + ' seconds before and ' + $("postSeconds").value + ' seconds after. Without a trigger, record until storage is full and save the full recording. A pulse near capacity may have an incomplete tail.';
}
for (const id of ["thresholdG", "preSeconds", "postSeconds"]) $(id).oninput = updateHint;
$("record").onclick = () => {
  try {
    const inputs = [$("preSeconds"), $("postSeconds"), $("thresholdG")];
    if (inputs.some(input => !input.reportValidity() || input.value === "")) throw Error("Enter valid trigger and window settings.");
    const [pre, post, threshold] = inputs.map(input => Number(input.value) * 1000);
    if (!pre && !post) throw Error("Choose a nonzero recording window.");
    if (threshold && threshold < 20) throw Error("Minimum automatic threshold is 0.02 g; use 0 for manual triggering.");
    const metadata = JSON.parse($("shotMetadata").value);
    if (!metadata || Array.isArray(metadata) || typeof metadata !== "object") throw Error("Metadata must be a JSON object.");
    if (new TextEncoder().encode(JSON.stringify(metadata)).length >= 512) throw Error("Test notes must be shorter than 512 UTF-8 bytes.");
    requestedPost = post / 1000;
    command("arm", {values:{pre_ms:Math.round(pre), post_ms:Math.round(post), threshold_mg:Math.round(threshold)}, metadata});
  } catch (err) { commandError = err.message; render(); }
};
$("trigger").onclick = () => command("trigger");
$("cancel").onclick = () => command("cancel");
$("delete").onclick = () => { if (confirm("Delete this recording from the board? Save it first if you want to keep it.")) command("delete"); };
function download(blob, filename) {
  const url = URL.createObjectURL(blob), link = document.createElement("a");
  link.href = url; link.download = filename; link.click(); setTimeout(() => URL.revokeObjectURL(url), 60000);
}
$("download").onclick = () => {
  if (rawBlob) download(rawBlob, 'bowvib_' + shot.id + '.bvr');
  else { const link = document.createElement("a"); link.href = "/shot.bin"; link.download = 'bowvib_' + shot.id + '.bvr'; link.click(); }
};
$("downloadNpz").onclick = () => { if (archiveBlob) download(archiveBlob, 'bowvib_' + loaded.id + '.npz'); };
async function loadRecording() {
  if (loading || !shot?.saved) return;
  loading = true; loadError = ""; render();
  try {
    const bytes = new Uint8Array(await readBoard("/shot.bin", "arrayBuffer", 60000));
    const parsed = format.readShot(bytes);
    if (parsed.id !== shot.id) throw Error("Stored recording changed. Refresh the page.");
    loaded = parsed; rawBlob = new Blob([bytes], {type:"application/octet-stream"});
    archiveBlob = format.shotArchive(parsed);
    fullStart = Math.min(...loaded.series.filter(s => s.count).map(s => s.start));
    fullEnd = Math.max(...loaded.series.filter(s => s.count).map(s => s.start + (s.count - 1) / s.rate));
    if (!(fullEnd > fullStart)) throw Error("Recording has too few samples to plot.");
    for (let i = 0; i < 4; i++) {
      $("sensor" + i).disabled = !loaded.counts[i];
      $("sensor" + i).checked = !!loaded.counts[i];
    }
    pointers.clear(); gesture = selection = null;
    viewStart = fullStart; viewEnd = fullEnd;
    if (loaded.triggered && $("autoFocus").checked) focusEvent(); else drawPlot();
  } catch (err) {
    loaded = rawBlob = archiveBlob = null; loadError = err.message;
    $("plotArea").hidden = true; $("savePlot").disabled = true;
  }
  finally { loading = false; render(); }
}
$("plot").onclick = () => { if (loaded) drawPlot(); else loadRecording(); };

function geometry() {
  const width = Math.max(200, $("chart").getBoundingClientRect().width || 900);
  const panels = $("plotMode").value === "magnitude" ? 1 : 3;
  const panelHeight = width < 600 ? 184 : 214;
  return {width, height: panels * panelHeight + 58, left: 62, right: width - 18, panels, panelHeight};
}
function valueAt(series, index, axis) {
  const offset = index * 6;
  if (axis < 3) return series.view.getInt16(offset + axis * 2, true) * .010417;
  const x = series.view.getInt16(offset, true), y = series.view.getInt16(offset + 2, true), z = series.view.getInt16(offset + 4, true);
  return Math.hypot(x, y, z) * .010417;
}
function timeLabel(seconds, span = viewEnd - viewStart) {
  return span < .2 ? (seconds * 1000).toFixed(span < .01 ? 2 : 1) + " ms" : seconds.toFixed(span < 2 ? 3 : 2) + " s";
}
function requestPlot() {
  if (!plotFrame) plotFrame = requestAnimationFrame(() => { plotFrame = 0; drawPlot(); });
}
function drawPlot() {
  if (!loaded) return;
  $("plotArea").hidden = false;
  const c = $("chart"), ctx = c.getContext("2d"), g = geometry();
  const {width, height, left, right, panels, panelHeight} = g, span = viewEnd - viewStart;
  const dpr = Math.min(globalThis.devicePixelRatio || 1, 3);
  c.style.height = height + "px";
  c.width = Math.round(width * dpr); c.height = Math.round(height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = "#111c29"; ctx.fillRect(0, 0, width, height);
  const visible = loaded.series.filter(s => s.count && $("sensor" + s.id).checked);
  const info = loaded.triggerInfo;
  const threshold = panels === 1 && info?.kind === "threshold" ? info.thresholdMg / 1000 : null;
  for (let panel = 0; panel < panels; panel++) {
    const axis = panels === 1 ? 3 : panel, top = 32 + panel * panelHeight, bottom = top + panelHeight - 50;
    let lo = Infinity, hi = -Infinity;
    const traces = visible.map(s => {
      const from = Math.max(0, Math.ceil((viewStart - s.start) * s.rate) - 1);
      const to = Math.min(s.count, Math.floor((viewEnd - s.start) * s.rate) + 2);
      const step = Math.max(1, Math.ceil((to - from) / (right - left))), points = [];
      // Preserve both extrema per pixel bucket: even a one-sample pulse stays visible.
      for (let i = from; i < to; i += step) {
        let min = Infinity, max = -Infinity, imin = i, imax = i;
        for (let j = i; j < Math.min(to, i + step); j++) {
          const v = valueAt(s, j, axis);
          if (v < min) { min = v; imin = j; }
          if (v > max) { max = v; imax = j; }
        }
        lo = Math.min(lo, min); hi = Math.max(hi, max);
        const indices = imin === imax ? [imin] : imin < imax ? [imin, imax] : [imax, imin];
        for (const j of indices) points.push([s.start + j / s.rate, valueAt(s, j, axis)]);
      }
      return {s, points};
    });
    if (!Number.isFinite(lo)) { lo = -1; hi = 1; }
    if (threshold !== null) { lo = Math.min(lo, 0); hi = Math.max(hi, threshold); }
    const pad = Math.max(.02, (hi - lo) * .12); lo -= pad; hi += pad;
    if (axis === 3) lo = 0;
    const xAt = t => left + (t - viewStart) / span * (right - left);
    const yAt = v => bottom - (v - lo) / (hi - lo) * (bottom - top);
    ctx.font = "600 12px system-ui"; ctx.fillStyle = "#e7edf7"; ctx.textAlign = "left";
    ctx.fillText(axis === 3 ? "XYZ magnitude · g" : "XYZ"[axis] + " axis · g", left, top - 13);
    ctx.font = "11px system-ui"; ctx.lineWidth = 1;
    const ticks = width < 500 ? 2 : 4;
    for (let k = 0; k <= 4; k++) {
      const value = lo + k / 4 * (hi - lo), y = yAt(value);
      ctx.strokeStyle = "#273445"; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
      ctx.fillStyle = "#9daec3"; ctx.textAlign = "right";
      ctx.fillText(value.toFixed(hi - lo < 4 ? 2 : 0), left - 9, y + 4);
    }
    for (let k = 0; k <= ticks; k++) {
      const t = viewStart + k / ticks * span, x = xAt(t);
      ctx.strokeStyle = "#202d3d"; ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, bottom); ctx.stroke();
      ctx.fillStyle = "#9daec3"; ctx.textAlign = k === 0 ? "left" : k === ticks ? "right" : "center";
      ctx.fillText(timeLabel(t, span), x, bottom + 20);
    }
    ctx.save(); ctx.beginPath(); ctx.rect(left, top, right - left, bottom - top); ctx.clip();
    if (lo < 0 && hi > 0) {
      ctx.strokeStyle = "#46556b"; ctx.beginPath(); ctx.moveTo(left, yAt(0)); ctx.lineTo(right, yAt(0)); ctx.stroke();
    }
    for (const {s, points} of traces) {
      ctx.strokeStyle = colors[s.id]; ctx.lineWidth = 1.3; ctx.beginPath();
      points.forEach(([t, v], i) => { if (!i) ctx.moveTo(xAt(t), yAt(v)); else ctx.lineTo(xAt(t), yAt(v)); });
      ctx.stroke();
    }
    if (threshold !== null) {
      ctx.strokeStyle = "#f9ca75"; ctx.setLineDash([3, 4]);
      ctx.beginPath(); ctx.moveTo(left, yAt(threshold)); ctx.lineTo(right, yAt(threshold)); ctx.stroke(); ctx.setLineDash([]);
      ctx.textAlign = "right"; ctx.fillStyle = "#f9ca75";
      ctx.fillText(threshold + " g threshold", right - 6, yAt(threshold) - 6);
    }
    if (loaded.triggered && viewStart <= 0 && viewEnd >= 0) {
      ctx.strokeStyle = "#e7edf7"; ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.moveTo(xAt(0), top); ctx.lineTo(xAt(0), bottom); ctx.stroke(); ctx.setLineDash([]);
      if (info?.kind === "threshold") {
        const s = visible.find(s => s.id === info.sensor);
        if (s) {
          const index = Math.round(-s.start * s.rate);
          if (index >= 0 && index < s.count) {
            ctx.fillStyle = colors[s.id]; ctx.beginPath(); ctx.arc(xAt(0), yAt(valueAt(s, index, axis)), 3.5, 0, 2 * Math.PI); ctx.fill();
          }
        }
      }
    }
    if (selection) {
      ctx.fillStyle = "#60a5fa30";
      ctx.fillRect(xAt(Math.min(...selection)), top, Math.abs(selection[1] - selection[0]) / span * (right - left), bottom - top);
    }
    ctx.restore();
    if (loaded.triggered && viewStart <= 0 && viewEnd >= 0) {
      ctx.textAlign = "center"; ctx.fillStyle = "#e7edf7";
      ctx.fillText("Trigger · 0", Math.max(left + 38, Math.min(right - 38, xAt(0))), top + 13);
    }
    if (!visible.length) {
      ctx.textAlign = "center"; ctx.fillStyle = "#9daec3";
      ctx.fillText("Select a sensor above to show its trace", (left + right) / 2, (top + bottom) / 2);
    }
  }
  ctx.font = "11px system-ui"; ctx.textAlign = "center";
  visible.forEach((s, i) => {
    ctx.fillStyle = colors[s.id];
    ctx.fillText("Sensor " + "ABCD"[s.id], width / 2 + (i - (visible.length - 1) / 2) * 56, height - 36);
  });
  ctx.textAlign = "center"; ctx.fillStyle = "#9daec3"; ctx.font = "12px system-ui";
  ctx.fillText(loaded.triggered ? "Time relative to trigger" : "Time from recording start", (left + right) / 2, height - 14);
  $("savePlot").disabled = false; $("focusEvent").disabled = !loaded.triggered;
  $("viewLabel").textContent = timeLabel(viewStart) + " to " + timeLabel(viewEnd) + " · " + timeLabel(span) + " visible";
  $("plotSummary").textContent = info?.kind === "threshold" ?
    "Time zero: first recorded sample reaching " + info.thresholdMg / 1000 + " g on sensor " + "ABCD"[info.sensor] + "." :
    loaded.triggered ? (info?.kind === "manual" ? "Time zero: manual trigger." : "Time zero: stored trigger time (legacy recording).") : "Full session saved without a trigger.";
  const room = Math.max(0, fullEnd - fullStart - span);
  $("pan").disabled = room < 1e-8;
  $("pan").value = room > 0 ? Math.round((viewStart - fullStart) / room * 1000) : 0;
  $("pan").setAttribute("aria-valuetext", timeLabel(viewStart) + " to " + timeLabel(viewEnd));
}
function setView(start, span, immediate = true) {
  if (!loaded) return;
  span = Math.min(fullEnd - fullStart, Math.max(.001, span));
  viewStart = Math.max(fullStart, Math.min(fullEnd - span, start)); viewEnd = viewStart + span;
  if (immediate) drawPlot(); else requestPlot();
}
function zoom(factor, anchor = (viewStart + viewEnd) / 2, immediate = true) {
  if (!loaded) return;
  const oldSpan = viewEnd - viewStart, span = Math.min(fullEnd - fullStart, Math.max(.001, oldSpan * factor));
  setView(anchor - span * (anchor - viewStart) / oldSpan, span, immediate);
}
function focusEvent() {
  if (!loaded?.triggered) return;
  const span = Math.min(1, fullEnd - fullStart);
  setView(-span / 2, span);
}
$("zoomIn").onclick = () => zoom(.5); $("zoomOut").onclick = () => zoom(2);
$("focusEvent").onclick = focusEvent;
$("resetZoom").onclick = () => setView(fullStart, fullEnd - fullStart);
$("pan").oninput = () => setView(fullStart + (fullEnd - fullStart - (viewEnd - viewStart)) * Number($("pan").value) / 1000, viewEnd - viewStart);
$("plotMode").onchange = drawPlot;
for (let i = 0; i < 4; i++) $("sensor" + i).onchange = drawPlot;
function pointerFraction(event) {
  const rect = $("chart").getBoundingClientRect(), g = geometry();
  return Math.max(0, Math.min(1, (event.clientX - rect.left - g.left) / (g.right - g.left)));
}
function pointerTime(event) { return viewStart + pointerFraction(event) * (viewEnd - viewStart); }
function beginGesture(select = false) {
  const points = [...pointers.values()]; selection = null;
  if (points.length >= 2) {
    const [a, b] = points, mid = (a.clientX + b.clientX) / 2;
    gesture = {kind: "pinch", span: viewEnd - viewStart,
      distance: Math.max(10, Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY)),
      anchor: pointerTime({clientX: mid})};
  } else if (points.length) {
    gesture = {kind: select ? "select" : "pan", x: points[0].clientX, start: viewStart, span: viewEnd - viewStart, time: pointerTime(points[0])};
  } else gesture = null;
}
const chart = $("chart");
chart.addEventListener("wheel", event => {
  if (!loaded) return;
  event.preventDefault();
  const scale = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? geometry().height : 1;
  if (!event.ctrlKey && Math.abs(event.deltaX) > Math.abs(event.deltaY)) {
    setView(viewStart + event.deltaX * scale / (geometry().right - geometry().left) * (viewEnd - viewStart), viewEnd - viewStart, false);
  } else zoom(Math.exp(Math.max(-1, Math.min(1, event.deltaY * scale * (event.ctrlKey ? .01 : .002)))), pointerTime(event), false);
}, {passive: false});
chart.onpointerdown = event => {
  if (!loaded || (event.button !== undefined && event.button !== 0)) return;
  event.preventDefault();
  pointers.set(event.pointerId, {clientX: event.clientX, clientY: event.clientY});
  chart.setPointerCapture(event.pointerId);
  beginGesture(event.shiftKey && event.pointerType !== "touch");
  chart.style.cursor = gesture.kind === "select" ? "crosshair" : "grabbing";
};
chart.onpointermove = event => {
  if (!pointers.has(event.pointerId) || !gesture) return;
  event.preventDefault();
  pointers.set(event.pointerId, {clientX: event.clientX, clientY: event.clientY});
  const points = [...pointers.values()], g = geometry();
  if (gesture.kind === "pinch" && points.length >= 2) {
    const [a, b] = points;
    const distance = Math.max(10, Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY));
    const span = Math.min(fullEnd - fullStart, Math.max(.001, gesture.span * gesture.distance / distance));
    setView(gesture.anchor - pointerFraction({clientX: (a.clientX + b.clientX) / 2}) * span, span, false);
  } else if (gesture.kind === "select") {
    selection = [gesture.time, pointerTime(event)]; requestPlot();
  } else if (gesture.kind === "pan") {
    setView(gesture.start - (event.clientX - gesture.x) / (g.right - g.left) * gesture.span, gesture.span, false);
  }
};
function endPointer(event, cancelled = false) {
  if (!pointers.has(event.pointerId)) return;
  if (!cancelled && gesture?.kind === "select" && Math.abs(event.clientX - gesture.x) > 8) {
    const end = pointerTime(event);
    setView(Math.min(gesture.time, end), Math.abs(end - gesture.time), false);
  }
  pointers.delete(event.pointerId);
  if (chart.hasPointerCapture(event.pointerId)) chart.releasePointerCapture(event.pointerId);
  beginGesture(); chart.style.cursor = pointers.size ? "grabbing" : "grab"; requestPlot();
}
chart.onpointerup = event => endPointer(event);
chart.onpointercancel = event => endPointer(event, true);
chart.onlostpointercapture = event => endPointer(event, true);
chart.ondblclick = () => { if (loaded?.triggered) focusEvent(); else $("resetZoom").onclick(); };
chart.onkeydown = event => {
  if (!loaded) return;
  if (event.key === "+" || event.key === "=") zoom(.5);
  else if (event.key === "-") zoom(2);
  else if (event.key === "ArrowLeft" || event.key === "ArrowRight") setView(viewStart + (event.key === "ArrowLeft" ? -.15 : .15) * (viewEnd - viewStart), viewEnd - viewStart);
  else if (event.key === "Home") $("resetZoom").onclick();
  else if (event.key.toLowerCase() === "e") focusEvent();
  else return;
  event.preventDefault();
};
if (globalThis.ResizeObserver) new ResizeObserver(requestPlot).observe(chart);
$("savePlot").onclick = () => {
  if (!loaded) return;
  drawPlot();
  chart.toBlob(blob => { if (blob) download(blob, 'bowvib_' + loaded.id + '_plot.png'); }, "image/png");
};
updateHint();
globalThis.BowVibBoot?.started();
setInterval(refresh, 750); refresh();
