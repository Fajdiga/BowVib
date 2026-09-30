"use strict";
const $ = id => document.getElementById(id);
const format = globalThis.BowVibFormat;
let socket, board, decoder, recording = false, starting = false;
let chunks = [[], [], [], []], counts = [0, 0, 0, 0], arrays, completed, archiveBlob;
let startedAt = 0, requestedAt = 0, captureTimestamp = "", error = "", wakeLock;

function expectedRates() {
  return board.freq_fine.map(trim => 7680 * (trim === -128 ? 1 : 1 + trim * 0.0013));
}
function sensorIds() {
  return [1, 2, 3, 4].filter(id => board.present_mask & (1 << (id - 1)));
}
function render() {
  $("record").disabled = recording || starting || !board || !board.present_mask || socket?.readyState !== WebSocket.OPEN;
  $("record").textContent = recording ? "Recording..." : "Start 10-second capture";
  $("download").disabled = !archiveBlob; $("plot").disabled = !archiveBlob;
  if (recording) {
    const elapsed = Math.min(10, (performance.now() - startedAt) / 1000);
    $("progress").value = elapsed;
    $("status").textContent = `Recording ${elapsed.toFixed(1)} / 10.0 s\nSamples A-D: ${counts.join(" / ")}`;
  } else if (error) $("status").textContent = error;
  else if (starting) $("status").textContent = "Starting sensors...";
  else if (completed) {
    $("progress").value = 10;
    $("status").textContent = `Capture complete (${completed.duration.toFixed(3)} s)\nSamples A-D: ${counts.join(" / ")}\nMeasured Hz: ${counts.map(n => (n / completed.duration).toFixed(1)).join(" / ")}\nTrim-predicted Hz: ${expectedRates().map(n => n.toFixed(1)).join(" / ")}\nNo overrun reported. Ready to save.`;
  } else if (board) $("status").textContent = `Ready\nDetected IMUs: ${sensorIds().join(", ") || "none"}\nConnected directly to BowVib over Wi-Fi.`;
}
function releaseWakeLock() {
  if (wakeLock) { wakeLock.release().catch(() => {}); wakeLock = null; }
}
function fail(message) {
  if (recording || starting) {
    try { socket?.send("STOP"); } catch (_) {}
    socket?.close();
  }
  recording = false; starting = false; archiveBlob = null;
  error = message; releaseWakeLock(); render();
}
function frameReceived(frame) {
  if (!recording) throw Error("Samples received before capture start");
  if (frame.type === "data") {
    if (!(board.present_mask & (1 << frame.sensor))) throw Error("Data from an absent IMU");
    counts[frame.sensor] += frame.rows; chunks[frame.sensor].push(frame.bytes); return;
  }
  if (frame.counts.some((n, i) => n !== counts[i])) throw Error("Frame/footer sample counts do not match");
  if (frame.mask !== 0) throw Error(`Capture lost data (status 0x${frame.mask.toString(16)}). Please record again.`);
  arrays = chunks.map(format.merge);
  archiveBlob = format.archive(arrays, sensorIds(), frame.duration, frame.mask, expectedRates());
  completed = frame; recording = false; starting = false; chunks = [[], [], [], []];
  releaseWakeLock(); render();
}
async function connect() {
  try {
    const response = await fetch("/status", {cache: "no-store"});
    if (!response.ok) throw Error("Cannot read board status");
    board = await response.json();
    socket = new WebSocket(`ws://${location.host}/ws`); socket.binaryType = "arraybuffer";
    socket.onopen = () => { if (!completed) error = ""; render(); };
    socket.onmessage = event => {
      try {
        if (typeof event.data === "string") {
          if (event.data === "CAPTURE START" && starting) {
            starting = false; recording = true; startedAt = performance.now(); render();
          } else if (event.data.startsWith("ERR ")) throw Error("Board: " + event.data);
          else throw Error("Unexpected board response");
        } else decoder.push(new Uint8Array(event.data));
      } catch (err) { fail(err.message); }
    };
    socket.onclose = () => {
      if (recording || starting) fail("Wi-Fi disconnected during capture. Please record again.");
      else if (!completed) { error = "Connecting to board... Close other capture tabs if this persists."; render(); }
      setTimeout(connect, 2000);
    };
    socket.onerror = () => { error = "Cannot connect. Stay connected to BowVib-IMU Wi-Fi."; render(); };
  } catch (err) {
    error = err.message + ". Connect to BowVib-IMU and keep this page open."; render();
    setTimeout(connect, 2000);
  }
}
async function startCapture() {
  if (recording || starting || socket?.readyState !== WebSocket.OPEN) return;
  chunks = [[], [], [], []]; counts = [0, 0, 0, 0]; arrays = null; completed = null; archiveBlob = null; error = "";
  captureTimestamp = new Date().toISOString().replace(/[:.]/g, "-");
  decoder = new format.StreamDecoder(frameReceived); starting = true; requestedAt = performance.now();
  $("progress").value = 0; $("chart").hidden = true; $("savePlot").disabled = true; render();
  // Screen Wake Lock may be unavailable over plain HTTP.
  try { if (navigator.wakeLock) wakeLock = await navigator.wakeLock.request("screen"); } catch (_) {}
  try { socket.send("START"); } catch (err) { fail(err.message); }
}
function download(blob, filename) {
  const url = URL.createObjectURL(blob), link = document.createElement("a");
  link.href = url; link.download = filename; document.body.appendChild(link); link.click(); link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 60000);
}
function plotCapture() {
  if (!arrays || !completed) return;
  const canvas = $("chart"), ctx = canvas.getContext("2d"), colors = ["#42a5f5", "#ff9f43", "#43c05b", "#ff5252"];
  const width = canvas.width, left = 85, right = 20, panel = 290, gap = 38;
  ctx.fillStyle = "#171e28"; ctx.fillRect(0, 0, width, canvas.height);
  ctx.fillStyle = "#e8edf3"; ctx.font = "20px system-ui"; ctx.fillText(`BowVib | ${completed.duration.toFixed(3)} s`, left, 30);
  const series = sensorIds().map(id => {
    const bytes = arrays[id - 1], view = new DataView(bytes.buffer, bytes.byteOffset, bytes.length);
    const count = counts[id - 1], points = Math.min(count, 2500);
    return {id, points: Array.from({length: points}, (_, i) => {
      const row = Math.round(i * (count - 1) / Math.max(1, points - 1));
      return [row / count * completed.duration, ...[0, 1, 2].map(axis => view.getInt16(row * 6 + axis * 2, true) * 0.010417)];
    })};
  });
  for (let axis = 0; axis < 3; axis++) {
    const top = 70 + axis * (panel + gap), bottom = top + panel - 25;
    let low = Infinity, high = -Infinity;
    for (const sensor of series) for (const point of sensor.points) { low = Math.min(low, point[axis + 1]); high = Math.max(high, point[axis + 1]); }
    const padding = Math.max(0.02, (high - low) * 0.08); low -= padding; high += padding;
    ctx.font = "17px system-ui"; ctx.fillStyle = "#b6c5d6"; ctx.strokeStyle = "#364252";
    for (let tick = 0; tick <= 4; tick++) {
      const fraction = tick / 4, y = bottom - fraction * (bottom - top);
      ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(width - right, y); ctx.stroke();
      ctx.fillText((low + fraction * (high - low)).toFixed(2), 12, y + 5);
    }
    ctx.fillText(`${"XYZ"[axis]} [g]`, 12, top - 12);
    for (const sensor of series) {
      ctx.strokeStyle = colors[sensor.id - 1]; ctx.lineWidth = 1.5; ctx.beginPath();
      sensor.points.forEach((point, i) => {
        const x = left + point[0] / completed.duration * (width - left - right);
        const y = bottom - (point[axis + 1] - low) / (high - low) * (bottom - top);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }); ctx.stroke();
    }
    ctx.lineWidth = 1;
    if (axis === 0) for (const sensor of series) { ctx.fillStyle = colors[sensor.id - 1]; ctx.fillText(`Sensor ${"ABCD"[sensor.id - 1]}`, left + (sensor.id - 1) * 210, 55); }
  }
  ctx.fillStyle = "#b6c5d6"; ctx.fillText(`Time: 0 to ${completed.duration.toFixed(3)} s`, left, canvas.height - 12);
  canvas.hidden = false; $("savePlot").disabled = false;
}
$("record").onclick = startCapture;
$("download").onclick = () => { if (archiveBlob) download(archiveBlob, `bowvib_capture_${captureTimestamp}.npz`); };
$("plot").onclick = plotCapture;
$("savePlot").onclick = () => $("chart").toBlob(blob => { if (blob) download(blob, `bowvib_plot_${captureTimestamp}.png`); }, "image/png");
setInterval(() => {
  if (starting && performance.now() - requestedAt > 5000) fail("Board did not start capture. Please try again.");
  if (recording && performance.now() - startedAt > 25000) fail("Capture did not complete. Please record again.");
  render();
}, 250);
connect();
