/* Binary stream validation and NumPy archive encoding. No network dependencies. */
(() => {
  "use strict";
  const encoder = new TextEncoder();

  class StreamDecoder {
    constructor(onFrame) {
      this.onFrame = onFrame;
      this.pending = new Uint8Array(0);
      this.ended = false;
    }
    push(bytes) {
      if (this.ended) throw Error("Data received after capture footer");
      const data = new Uint8Array(this.pending.length + bytes.length);
      data.set(this.pending); data.set(bytes, this.pending.length);
      let offset = 0;
      while (data.length - offset >= 4) {
        const magic = String.fromCharCode(...data.subarray(offset, offset + 4));
        if (magic === "IM4D") {
          if (data.length - offset < 7) break;
          const view = new DataView(data.buffer, offset);
          const sensor = view.getUint8(4), rows = view.getUint16(5, true);
          if (sensor >= 4 || rows < 1 || rows > 256) throw Error("Invalid sample frame");
          const length = 7 + rows * 6;
          if (data.length - offset < length) break;
          this.onFrame({type: "data", sensor, rows, bytes: data.slice(offset + 7, offset + length)});
          offset += length;
        } else if (magic === "IM4E") {
          if (data.length - offset < 25) break;
          const view = new DataView(data.buffer, offset, 25);
          const counts = Array.from({length: 4}, (_, i) => view.getUint32(4 + i * 4, true));
          const duration = view.getUint32(20, true) / 1000;
          if (!(duration > 0)) throw Error("Invalid capture duration");
          this.ended = true;
          this.onFrame({type: "end", counts, duration, mask: view.getUint8(24)});
          offset += 25;
          if (offset !== data.length) throw Error("Extra data after capture footer");
        } else throw Error("Invalid capture stream");
      }
      this.pending = data.slice(offset);
    }
  }
  function merge(parts) {
    const data = new Uint8Array(parts.reduce((total, part) => total + part.length, 0));
    let offset = 0;
    for (const part of parts) { data.set(part, offset); offset += part.length; }
    return data;
  }
  function npy(bytes, dtype, shape) {
    const dimensions = shape.length ? shape.join(", ") + (shape.length === 1 ? "," : "") : "";
    let header = `{'descr': '${dtype}', 'fortran_order': False, 'shape': (${dimensions}), }`;
    header += " ".repeat((64 - ((10 + header.length + 1) % 64)) % 64) + "\n";
    const prefix = new Uint8Array(10);
    prefix.set([0x93, 78, 85, 77, 80, 89, 1, 0]);
    new DataView(prefix.buffer).setUint16(8, header.length, true);
    return merge([prefix, encoder.encode(header), bytes]);
  }
  function numbers(values, dtype, scalar = false) {
    const formats = {"|u1": [1, "setUint8"], "<u2": [2, "setUint16"], "<u4": [4, "setUint32"], "<f8": [8, "setFloat64"]};
    const [size, setter] = formats[dtype];
    const bytes = new Uint8Array(values.length * size), view = new DataView(bytes.buffer);
    values.forEach((value, i) => view[setter](i * size, value, true));
    return npy(bytes, dtype, scalar ? [] : [values.length]);
  }
  const crcTable = new Uint32Array(256);
  for (let i = 0; i < 256; i++) {
    let crc = i;
    for (let bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    crcTable[i] = crc >>> 0;
  }
  function crc32(bytes) {
    let crc = 0xffffffff;
    for (const byte of bytes) crc = (crc >>> 8) ^ crcTable[(crc ^ byte) & 255];
    return (crc ^ 0xffffffff) >>> 0;
  }
  function zip(files) {
    const localParts = [], directory = [];
    let offset = 0, directoryLength = 0;
    for (const [name, bytes] of files) {
      const filename = encoder.encode(name), crc = crc32(bytes);
      const local = new Uint8Array(30), lv = new DataView(local.buffer);
      lv.setUint32(0, 0x04034b50, true); lv.setUint16(4, 20, true);
      lv.setUint16(12, 33, true);
      lv.setUint32(14, crc, true); lv.setUint32(18, bytes.length, true);
      lv.setUint32(22, bytes.length, true); lv.setUint16(26, filename.length, true);
      localParts.push(local, filename, bytes);
      const central = new Uint8Array(46), cv = new DataView(central.buffer);
      cv.setUint32(0, 0x02014b50, true); cv.setUint16(4, 20, true); cv.setUint16(6, 20, true);
      cv.setUint16(14, 33, true); cv.setUint32(16, crc, true);
      cv.setUint32(20, bytes.length, true); cv.setUint32(24, bytes.length, true);
      cv.setUint16(28, filename.length, true); cv.setUint32(42, offset, true);
      directory.push(central, filename); directoryLength += central.length + filename.length;
      offset += local.length + filename.length + bytes.length;
    }
    const end = new Uint8Array(22), ev = new DataView(end.buffer);
    ev.setUint32(0, 0x06054b50, true); ev.setUint16(8, files.length, true); ev.setUint16(10, files.length, true);
    ev.setUint32(12, directoryLength, true); ev.setUint32(16, offset, true);
    return new Blob([...localParts, ...directory, end], {type: "application/octet-stream"});
  }
  function archive(arrays, ids, duration, mask, expectedRates) {
    if (!Number.isFinite(duration) || duration <= 0 || !ids.length || new Set(ids).size !== ids.length || ids.some(i => i < 1 || i > 4)) {
      throw Error("Invalid capture metadata");
    }
    const counts = arrays.map(bytes => bytes.length / 6);
    if (counts.some(n => !Number.isInteger(n)) || ids.some(i => counts[i - 1] < 2)) throw Error("Missing or incomplete sensor samples");
    const rates = counts.map(count => count / duration);
    const files = arrays.map((bytes, i) => [`imu${i + 1}_samples_lsb.npy`, npy(bytes, "<i2", [counts[i], 3])]);
    const fields = [
      ["sensor_counts", counts, "<u4"], ["sensor_sample_rates_hz", rates, "<f8"],
      ["sensor_expected_sample_rates_hz", expectedRates, "<f8"], ["sensor_ids", ids, "|u1"],
      ["sample_rate_hz", [ids.reduce((total, id) => total + rates[id - 1], 0) / ids.length], "<f8", true],
      ["capture_duration_s", [duration], "<f8", true], ["configured_sample_rate_hz", [7680], "<u4", true],
      ["full_scale_g", [320], "<u2", true], ["output_bits", [16], "<u2", true],
      ["overrun_mask", [mask], "|u1", true], ["overrun_mask_valid", [1], "|u1", true],
    ];
    for (const [name, values, dtype, scalar] of fields) files.push([`${name}.npy`, numbers(values, dtype, scalar)]);
    return zip(files);
  }
  globalThis.BowVibFormat = {StreamDecoder, merge, archive};
})();
