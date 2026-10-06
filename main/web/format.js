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
  function hardwareTimeFit(anchors, clocks, origin, dense=false) {
    if (!anchors.length || clocks.length < 2) throw Error("Missing hardware timestamp or clock calibration records");
    const mean = values => values.reduce((a,b)=>a+b,0)/values.length;
    for(let i=1;i<anchors.length;i++) {
      const dx=anchors[i].index-anchors[i-1].index,dy=anchors[i].ticks-anchors[i-1].ticks;
      if(dx<=0 || dy<=0 || dy-6*dx < -2) throw Error("Inconsistent hardware timestamp/sample sequence");
    }
    const mx=mean(anchors.map(a=>a.index)),my=mean(anchors.map(a=>a.ticks));
    let xx=0,xy=0;
    for(const a of anchors){const dx=a.index-mx;xx+=dx*dx;xy+=dx*(a.ticks-my)}
    const tickSlope=xx ? xy/xx : 6,tickIntercept=mean(anchors.map(a=>a.ticks-tickSlope*a.index));
    const intervals=anchors.slice(1).map((a,i)=>a.ticks-anchors[i].ticks-6*(a.index-anchors[i].index));
    const fifoError=intervals.reduce((m,x)=>Math.max(m,Math.abs(x)),0);
    const discontinuities=intervals.filter(x=>Math.abs(x)>2).length;
    const minSpan=Math.min(...clocks.map(c=>c.span));
    const good=clocks.filter(c=>c.span<=minSpan+50);
    if(good.length<2)throw Error("Insufficient short-bracket clock calibration records");
    for(let i=1;i<good.length;i++)if(good[i].ticks<=good[i-1].ticks || good[i].midpoint<=good[i-1].midpoint)throw Error("Non-monotonic sensor clock calibration");
    const weights=good.map(c=>1/Math.max(c.span,1)**2),ws=weights.reduce((a,b)=>a+b,0);
    const cx=good.reduce((n,c,i)=>n+weights[i]*c.ticks,0)/ws;
    const cy=good.reduce((n,c,i)=>n+weights[i]*(c.midpoint-origin)/1e6,0)/ws;
    xx=xy=0;
    good.forEach((c,i)=>{const dx=c.ticks-cx;xx+=weights[i]*dx*dx;xy+=weights[i]*dx*((c.midpoint-origin)/1e6-cy)});
    const clockSlope=xy/xx,clockIntercept=cy-clockSlope*cx;
    if(!(1/(46080*1.2)<clockSlope && clockSlope<1/(46080*.8)))throw Error("Implausible sensor timestamp clock rate");
    const residuals=good.map(c=>(c.midpoint-origin)/1e6-clockIntercept-clockSlope*c.ticks);
    const maxError=Math.max(...residuals.map(x=>Math.abs(x)))*1e6;
    let localError=0;
    for(let i=1;i<good.length-1;i++) {
      const a=good[i-1],b=good[i+1],c=good[i];
      const predicted=a.midpoint+(c.ticks-a.ticks)*(b.midpoint-a.midpoint)/(b.ticks-a.ticks);
      localError=Math.max(localError,Math.abs(c.midpoint-predicted));
    }
    const uncertainty=localError+Math.max(...good.map(c=>c.span))/2+clockSlope*1e6*(1+(dense?0:fifoError));
    const slope=tickSlope*clockSlope;
    const timeAt=index=>{
      let lo=0,hi=anchors.length;
      while(lo<hi){const mid=(lo+hi)>>1;if(anchors[mid].index<index)lo=mid+1;else hi=mid}
      let ticks;
      if(lo===0)ticks=anchors[0].ticks+6*(index-anchors[0].index);
      else if(lo===anchors.length)ticks=anchors.at(-1).ticks+6*(index-anchors.at(-1).index);
      else {const a=anchors[lo-1],b=anchors[lo];ticks=a.ticks+(index-a.index)*(b.ticks-a.ticks)/(b.index-a.index)}
      lo=0;hi=good.length;
      while(lo<hi){const mid=(lo+hi)>>1;if(good[mid].ticks<ticks)lo=mid+1;else hi=mid}
      if(lo===0)return (good[0].midpoint-origin)/1e6+(ticks-good[0].ticks)*clockSlope;
      if(lo===good.length)return (good.at(-1).midpoint-origin)/1e6+(ticks-good.at(-1).ticks)*clockSlope;
      const a=good[lo-1],b=good[lo];
      return (a.midpoint-origin)/1e6+(ticks-a.ticks)*(b.midpoint-a.midpoint)/1e6/(b.ticks-a.ticks);
    };
    return {slope,intercept:clockIntercept+tickIntercept*clockSlope,timeAt,diagnostic:{
      clock_samples:good.length,fifo_timestamp_count:anchors.length,clock_tick_us:clockSlope*1e6,
      clock_fit_rms_us:Math.sqrt(mean(residuals.map(x=>x*x)))*1e6,clock_fit_max_us:maxError,
      clock_interpolation_max_us:localError,
      fifo_fit_max_ticks:fifoError,estimated_timing_uncertainty_us:uncertainty,sample_period_us:slope*1e6,timestamp_discontinuities:discontinuities}};
  }
  function readShot(bytes) {
    if (bytes.length < 608) throw Error("Truncated recording manifest");
    const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.length);
    const magic = offset => String.fromCharCode(...bytes.subarray(offset, offset + 4));
    const u32 = offset => v.getUint32(offset, true);
    const u64 = offset => {
      const n = Number(v.getBigUint64(offset, true));
      if (!Number.isSafeInteger(n)) throw Error("Invalid timestamp");
      return n;
    };
    const version = u32(4);
    if (magic(0) !== "BVR1" || ![1,2,3,4,5].includes(version) || crc32(bytes.subarray(0,604)) !== u32(604)) throw Error("Invalid manifest or CRC");
    const first = u32(8), end = u32(12), capacity = u32(16), pre = u32(20), post = u32(24);
    const present = u32(36), quality = u32(40), saturation = u32(44);
    const started = u64(48), trigger = u64(56), ended = u64(64), origin = trigger || started;
    if (!(first < end && end-first <= capacity && capacity <= 2048) || bytes.length !== 608+(end-first)*4096) throw Error("Invalid page range or truncated recording");
    if (u32(28) !== 7680 || u32(32) !== 320 || !present || present > 15 || pre > 10000 || post > 3000 || pre+post === 0) throw Error("Unsupported acquisition configuration");
    if (ended < started || (trigger && !(started <= trigger && trigger <= ended))) throw Error("Invalid recording timestamps");
    const metadataBytes = bytes.subarray(92,604), terminator = metadataBytes.indexOf(0);
    if (terminator < 0) throw Error("Invalid metadata termination");
    const text = new TextDecoder("utf-8", {fatal:true}).decode(metadataBytes.subarray(0,terminator));
    let test; try { test = JSON.parse(text); } catch (_) { test = {raw_text:text}; }
    const id = Array.from(bytes.subarray(72,88), n => n.toString(16).padStart(2,"0")).join("");
    const trims = Array.from({length:4}, (_,i) => v.getInt8(88+i));
    const expected = trims.map(n => 7680*(n === -128 ? 1 : 1+n*.0013));
    const records = [[],[],[],[]], next = [null,null,null,null];
    let triggerInfo = version >= 2 && trigger ? {kind:"manual"} : null;
    const anchors=[[],[],[],[]],clocks=[[],[],[],[]],timingDiagnostics=[];
    let lastDataRecord = null;
    let lastPageTime = started;
    for (let seq = first; seq < end; seq++) {
      const base = 608+(seq-first)*4096, used = u32(base+8);
      if (magic(base) !== "BVP1" || u32(base+4) !== seq || used <= 32 || used > 4096) throw Error("Invalid page header or sequence");
      const checked = bytes.slice(base,base+used); checked.fill(0,12,16);
      if (crc32(checked) !== u32(base+12)) throw Error("Data page CRC mismatch");
      const firstTime = u64(base+16), lastTime = u64(base+24);
      if (!(lastPageTime <= firstTime && firstTime <= lastTime && lastTime <= ended)) throw Error("Non-monotonic page timestamps");
      let offset = base+32, previous = firstTime, recordFirst = null;
      while (offset < base+used) {
        if (base+used-offset < 19) throw Error("Truncated sample record");
        const drain = u64(offset), index = u32(offset+8), sensor = v.getUint8(offset+16), rows = v.getUint16(offset+17,true);
        if (drain < previous || drain > lastTime) throw Error("Invalid sample timestamp");
        if (["IM4H","IM4C"].includes(magic(offset+12))) {
          const hardware=magic(offset+12)==="IM4H",length=hardware?23:35;
          if(![3,4].includes(version) || (hardware && version!==3) || rows || sensor>=4 || !(present&(1<<sensor)) || offset+length>base+used)throw Error("Invalid hardware timing record");
          const ticks=u32(offset+19);
          if(hardware)anchors[sensor].push({index,ticks});
          else {
            const midpoint=u64(offset+23),span=u32(offset+31);
            if(index || midpoint<started || midpoint>drain || !span || span>1000000)throw Error("Invalid clock calibration bracket");
            clocks[sensor].push({ticks,midpoint,span});
          }
          previous=drain;recordFirst??=drain;offset+=length;continue;
        }
        if (magic(offset+12) === "IM4T") {
          if (version < 2 || !trigger || rows || sensor >= 4 || !(present & (1 << sensor)) || offset+23 > base+used || triggerInfo.kind !== "manual") throw Error("Invalid threshold trigger record");
          const thresholdMg = u32(offset+19);
          if (thresholdMg < 20 || thresholdMg > 320000) throw Error("Invalid threshold trigger value");
          if (!lastDataRecord || lastDataRecord.sensor !== sensor || lastDataRecord.drain !== drain || index < lastDataRecord.index || index >= lastDataRecord.index+lastDataRecord.rows) throw Error("Invalid threshold trigger sample reference");
          triggerInfo = {kind:"threshold",sensor,index,thresholdMg};
          previous = drain; recordFirst ??= drain; offset += 23;
          continue;
        }
        const timed=magic(offset+12)==="IM4S" && version>=4,length=timed?23+rows*7:19+rows*6;
        if ((!timed && (magic(offset+12)!=="IM4D" || version>=4)) || sensor >= 4 || !(present & (1 << sensor)) || rows < 1 || rows > 256 || offset+length > base+used) throw Error("Invalid sample frame");
        if (drain < previous || drain > lastTime || (next[sensor] !== null && index !== next[sensor])) throw Error("Invalid sample timestamp or sequence");
        next[sensor] = index+rows; previous = drain; recordFirst ??= drain;
        lastDataRecord = {sensor,index,rows,drain};
        let samples=bytes.subarray(offset+19,offset+19+rows*6),sensorTicks=null;
        if(timed) {
          samples=new Uint8Array(rows*6);sensorTicks=new Uint32Array(rows);let ticks=u32(offset+19);
          for(let row=0;row<rows;row++) {
            const at=offset+23+row*7,delta=v.getUint8(at+6);
            if((row===0 && delta!==0) || (row>0 && delta===0))throw Error("Invalid per-sample timestamp delta");
            ticks+=delta;if(ticks>0xffffffff)throw Error("Sensor timestamp rollover");
            if(version===5 && ticks>drain-started)throw Error("Sample time is after its storage time");
            sensorTicks[row]=ticks;
            anchors[sensor].push({index:index+row,ticks});samples.set(bytes.subarray(at,at+6),row*6);
          }
        }
        records[sensor].push({index,rows,drain:(drain-origin)/1e6,bytes:samples,sensorTicks});
        offset += length;
      }
      if (recordFirst !== firstTime || previous !== lastTime) throw Error("Page timestamp bounds disagree with records");
      lastPageTime = lastTime;
    }
    const warnings = [], rates = [], series = [], arrays = [], counts = [], fits = [];
    const windowStart = trigger ? -pre/1000 : 0, windowEnd = trigger ? post/1000 : (ended-started)/1e6;
    for (let sensor = 0; sensor < 4; sensor++) {
      const rs = records[sensor];
      let slope = 1/expected[sensor], intercept = (started-origin)/1e6;
      // Match the host's batch-end timing fit, including merged equal drain timestamps.
      const endpoints = rs.filter((r,i) => i === rs.length-1 || r.drain !== rs[i+1].drain);
      let diagnostic=null,timeAt=null;
      if(version===5 && (present&(1<<sensor))) {
        const a=anchors[sensor];
        if(!a.length || a.some((p,i)=>i && p.ticks<=a[i-1].ticks))throw Error("Missing or non-monotonic direct sample timestamps");
        let min=Infinity,max=0,gaps=0;
        for(let i=1;i<a.length;i++){const dt=a[i].ticks-a[i-1].ticks;min=Math.min(min,dt);max=Math.max(max,dt);if(dt>200)gaps++}
        if(a.length>1)slope=(a.at(-1).ticks-a[0].ticks)/(a.length-1)/1e6;
        const low=a[0].index;
        timeAt=index=>(a[index-low].ticks+started-origin)/1e6;
        diagnostic={timestamp_source:"ESP32 data-ready ISR",timestamp_count:a.length,timer_resolution_us:1,
          sample_period_us:slope*1e6,min_interval_us:a.length>1?min:null,max_interval_us:a.length>1?max:null,timestamp_discontinuities:gaps};
        if(gaps)warnings.push('Sensor '+(sensor+1)+': '+gaps+' data-ready intervals exceed 200 us; review possible missing sample');
        if(!(.8*7680<1/slope && 1/slope<1.2*7680))warnings.push('Sensor '+(sensor+1)+': estimated rate outside plausible range');
      } else if(version>=3 && (present&(1<<sensor))) {
        if(!rs.length)throw Error("Missing hardware-timed samples");
        const low=rs[0].index,high=rs.at(-1).index+rs.at(-1).rows-1;
        if(anchors[sensor].some(a=>a.index>high+1))throw Error("Hardware timestamp refers to missing sample");
        const fit=hardwareTimeFit(anchors[sensor].filter(a=>low<=a.index && a.index<=high),clocks[sensor],origin,version===4);
        slope=fit.slope;intercept=fit.intercept;diagnostic=fit.diagnostic;timeAt=fit.timeAt;
        if(diagnostic.estimated_timing_uncertainty_us>1000)warnings.push('Sensor '+(sensor+1)+': hardware timing uncertainty exceeds 1 ms');
        if(diagnostic.timestamp_discontinuities)warnings.push('Sensor '+(sensor+1)+': '+diagnostic.timestamp_discontinuities+' hardware timestamp interval discontinuities; review possible missing sample/time slot');
      } else if (endpoints.length > 1) {
        const meanX = endpoints.reduce((s,r) => s+r.index+r.rows-1,0)/endpoints.length;
        const meanY = endpoints.reduce((s,r) => s+r.drain,0)/endpoints.length;
        let xx=0,xy=0;
        for (const r of endpoints) { const dx=r.index+r.rows-1-meanX;xx+=dx*dx;xy+=dx*(r.drain-meanY); }
        if (xx > 0 && xy > 0) {
          slope=xy/xx;
          const offsets=endpoints.map(r=>r.drain-slope*(r.index+r.rows-1)).sort((a,b)=>a-b);
          const mid=Math.floor(offsets.length/2);intercept=offsets.length%2 ? offsets[mid] : (offsets[mid-1]+offsets[mid])/2;
          if (!(.8*7680 < 1/slope && 1/slope < 1.2*7680)) warnings.push('Sensor '+(sensor+1)+': estimated rate outside plausible range');
        }
      }
      rates.push(1/slope);
      fits.push({slope,intercept,timeAt});
      timingDiagnostics.push(diagnostic);
    }
    // Anchor every fitted sensor clock to the acquired threshold sample. This
    // removes FIFO/processing latency without claiming hardware synchronization.
    let alignmentShift = 0;
    if (triggerInfo?.kind === "threshold") {
      const {sensor,index} = triggerInfo, rs = records[sensor];
      if (!rs.length || index < rs[0].index || index >= rs.at(-1).index+rs.at(-1).rows) throw Error("Threshold trigger sample is missing");
      alignmentShift = fits[sensor].timeAt ? fits[sensor].timeAt(index) : fits[sensor].intercept+index*fits[sensor].slope;
    }
    for (let sensor = 0; sensor < 4; sensor++) {
      const rs = records[sensor], slope = fits[sensor].slope;
      const intercept = triggerInfo?.kind === "threshold" && triggerInfo.sensor === sensor
        ? -triggerInfo.index*slope : fits[sensor].intercept-alignmentShift;
      const parts=[],sampleTimes=[],sampleTicks=[]; let start=0, count=0;
      for(const r of rs) {
        if(fits[sensor].timeAt) {
          let from=-1,to=0;
          for(let i=0;i<r.rows;i++) {
            const t=fits[sensor].timeAt(r.index+i)-alignmentShift;
            if(t>=windowStart-1e-6 && t<=windowEnd+1e-6) {if(from<0)from=i;to=i+1;sampleTimes.push(t);if(r.sensorTicks)sampleTicks.push(r.sensorTicks[i])}
          }
          if(from>=0){if(!count)start=sampleTimes[0];parts.push(r.bytes.subarray(from*6,to*6));count+=to-from}
          continue;
        }
        const from=Math.max(0,Math.ceil((windowStart-1e-6-intercept)/slope-r.index));
        const to=Math.min(r.rows,Math.floor((windowEnd+1e-6-intercept)/slope-r.index)+1);
        if(to>from) {
          if(!count)start=triggerInfo?.kind === "threshold" && triggerInfo.sensor === sensor
            ? (r.index+from-triggerInfo.index)*slope : intercept+(r.index+from)*slope;
          parts.push(r.bytes.subarray(from*6,to*6));count+=to-from;
        }
      }
      const raw=merge(parts);arrays.push(raw);counts.push(count);
      if(present & (1<<sensor)) {
        if(count<2)warnings.push('Sensor '+(sensor+1)+': missing samples');
        else if(start>windowStart+.04 || (sampleTimes.length ? sampleTimes.at(-1) : start+(count-1)*slope)<windowEnd-.04)warnings.push('Sensor '+(sensor+1)+': requested window coverage incomplete');
        series.push({id:sensor,count,start,rate:1/slope,times:sampleTimes.length ? Float64Array.from(sampleTimes) : null,sensorTicks:sampleTicks.length ? Uint32Array.from(sampleTicks) : null,view:new DataView(raw.buffer,raw.byteOffset,raw.length)});
      }
    }
    if(quality)warnings.push('Quality mask 0x'+quality.toString(16)+(quality&32 ? ' (missing pre-trigger history)' : ''));
    if(saturation)warnings.push('Saturation mask 0x'+saturation.toString(16));
    const actualStart=Math.min(...series.filter(s=>s.count).map(s=>s.start));
    const actualEnd=Math.max(...series.filter(s=>s.count).map(s=>(s.times ? s.times.at(-1) : s.start+(s.count-1)/s.rate)+1/s.rate));
    return {id,triggered:!!trigger,triggerInfo,duration:actualEnd-actualStart,counts,rates,expected,series,arrays,warnings,
      metadata:{id,format_version:version,pre_ms:pre,post_ms:post,odr_hz:7680,fs_g:320,present_mask:present,quality_mask:quality,saturation_mask:saturation,
        started_us:started,trigger_us:trigger,ended_us:ended,test,triggered:!!trigger,trigger_info:triggerInfo,trigger_alignment_shift_s:alignmentShift,warnings,
        timing:triggerInfo?.kind === "threshold"
          ? "Zero is the recorded threshold sample; other sample times estimated by fitting software FIFO drain timestamps; no verified hardware sync"
          : "Estimated from software FIFO drain timestamps; no verified hardware sync",
        ...(version>=3 ? {timing:version===5 ? "Original ESP32 data-ready ISR timestamps; shared clock, independent sample edges; interrupt latency and physical pulse skew unverified" : "FIFO hardware timestamps mapped to the shared ESP32 clock; independent sample clocks; physical pulse skew unverified",timing_diagnostics:timingDiagnostics} : {})}};
  }
  function shotArchive(shot) {
    const ids=shot.series.map(s=>s.id+1);
    if (!Number.isFinite(shot.duration) || shot.duration <= 0 || ids.some(id => shot.counts[id-1] < 2)) throw Error("Missing or incomplete sensor samples");
    // Rebuild with timing and shot metadata to preserve the pulse-relative time axis.
    const files=shot.arrays.map((bytes,i)=>['imu'+(i+1)+'_samples_lsb.npy',npy(bytes,'<i2',[shot.counts[i],3])]);
    for(const s of shot.series) files.push(['imu'+(s.id+1)+'_time_s.npy',numbers(s.times || Array.from({length:s.count},(_,i)=>s.start+i/s.rate),'<f8')]);
    for(const s of shot.series) if(s.sensorTicks)files.push(['imu'+(s.id+1)+(shot.metadata.format_version===5 ? '_drdy_time_us.npy' : '_sensor_ticks.npy'),numbers(s.sensorTicks,'<u4')]);
    const text=JSON.stringify(shot.metadata), utf8=encoder.encode(text);
    files.push(['shot_metadata_json.npy',npy(utf8,'|S'+utf8.length,[])]);
    // Copy the small standard metadata entries from the archive's creation logic.
    const fields=[['sensor_counts',shot.counts,'<u4'],['sensor_sample_rates_hz',shot.rates,'<f8'],['sensor_expected_sample_rates_hz',shot.expected,'<f8'],['sensor_ids',ids,'|u1'],
      ['sample_rate_hz',[ids.reduce((n,id)=>n+shot.rates[id-1],0)/ids.length],'<f8',true],['capture_duration_s',[shot.duration],'<f8',true],
      ['configured_sample_rate_hz',[7680],'<u4',true],['full_scale_g',[320],'<u2',true],['output_bits',[16],'<u2',true],
      ['overrun_mask',[shot.metadata.quality_mask],'|u1',true],['overrun_mask_valid',[1],'|u1',true]];
    for(const [name,values,dtype,scalar] of fields)files.push([name+'.npy',numbers(values,dtype,scalar)]);
    return zip(files);
  }
  globalThis.BowVibFormat = {StreamDecoder, merge, archive, readShot, shotArchive};
})();
