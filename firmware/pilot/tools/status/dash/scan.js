// The wire format of src/scanwire.hxx, parsed once per new revolution into
// typed arrays the two views read in place - they draw at 10 Hz and their
// point loops must not allocate. F <n> <mHz> a,d,q ... then, while a pilot is
// deciding, D <mode> <clear> <hits> <steer> <throttle> <stop>. A frame whose
// count does not match is BAD, not shorter - the same rule as the C++ reader.

export const MAX_SAMPLES = 8192;    // the C1 gives ~500 a revolution; room for a denser lidar

export function makeScan() {
  return {
    n: 0,
    a: new Float32Array(MAX_SAMPLES),   // bearing, radians, 0 = the front, clockwise
    d: new Float32Array(MAX_SAMPLES),   // range, mm; 0 mm is no return and is dropped here
    hz: 0,
    bad: false,
    near: -1,       // index of the nearest return, -1 when there is none
    far: 0,         // the farthest, mm
    drive: null,    // the D line as {mode, clearMm, hits, steer, throttle, stop}, or null
    driveBuf: { mode: '', clearMm: 0, hits: 0, steer: 0, throttle: 0, stop: false },
  };
}

export function parseScan(text, out) {
  out.n = 0; out.hz = 0; out.bad = false; out.near = -1; out.far = 0; out.drive = null;
  const lines = text.split('\n');
  for (let li = 0; li < lines.length; li++) {
    const w = lines[li].trim().split(' ');
    if (w[0] === 'F') {
      out.hz = Number(w[2]) / 1000;
      if (w.length - 3 !== Number(w[1])) { out.bad = true; continue; }
      let nearD = Infinity;
      for (let k = 3; k < w.length && out.n < MAX_SAMPLES; k++) {
        const s = w[k];
        const c1 = s.indexOf(','), c2 = s.indexOf(',', c1 + 1);
        const d = Number(s.slice(c1 + 1, c2 < 0 ? s.length : c2));
        if (!(d > 0)) continue;
        const i = out.n++;
        out.a[i] = Number(s.slice(0, c1)) * (Math.PI / 18000);   // centi-degrees to radians
        out.d[i] = d;
        if (d < nearD) { nearD = d; out.near = i; }
        if (d > out.far) out.far = d;
      }
    } else if (w[0] === 'D' && w.length >= 7) {
      const dr = out.driveBuf;
      dr.mode = w[1]; dr.clearMm = Number(w[2]); dr.hits = Number(w[3]);
      dr.steer = Number(w[4]) / 1000; dr.throttle = Number(w[5]) / 1000;
      dr.stop = w[6] === '1';
      out.drive = dr;
    }
  }
  return out;
}
