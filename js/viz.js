// 캔버스 시각화: 시계열 차트 + 궤적 뷰 (외부 라이브러리 없이, 오프라인 동작)
export const COL = {
  grid: '#1c2632', axis: '#2a3746', text: '#8796a8', kf: '#4cc2ff', band: 'rgba(76,194,255,.16)',
  gps: '#3ddc97', ref: '#8796a8', out: '#ffb84d', outBg: 'rgba(255,184,77,.10)', err: '#ff6b6b', imu: '#c792ea',
};

function setup(cv) {
  const r = cv.getBoundingClientRect(), d = devicePixelRatio || 1;
  const W = Math.max(1, r.width), H = Math.max(1, r.height);
  if (cv.width !== Math.round(W * d) || cv.height !== Math.round(H * d)) { cv.width = Math.round(W * d); cv.height = Math.round(H * d); }
  const g = cv.getContext('2d');
  g.setTransform(d, 0, 0, d, 0, 0); g.clearRect(0, 0, W, H);
  g.font = '11px system-ui,-apple-system,sans-serif'; g.lineJoin = 'round'; g.lineCap = 'round';
  return [g, W, H];
}

function niceStep(range, n) {
  const raw = range / n, p = 10 ** Math.floor(Math.log10(raw)), m = raw / p;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
}

// series: { pts:[[t,y]] 또는 band는 [[t,lo,hi]], type:'line'|'dots'|'band', color, hollow:[[t,y]], wrap:true(0~360 끊김 처리) }
export function timeChart(cv, { t0, t1, series, outages = [], na = [], yMin, yMax, fmt = v => v }) {
  const [g, W, H] = setup(cv);
  const L = 34, Rr = 8, T = 8, B = 18, pw = W - L - Rr, ph = H - T - B;
  const inWin = p => p[0] >= t0 && p[0] <= t1;
  let lo = yMin, hi = yMax;
  if (lo == null || hi == null) {
    let a = Infinity, b = -Infinity;
    for (const s of series) for (const p of [...s.pts, ...(s.hollow || [])]) if (inWin(p)) for (let i = 1; i < p.length; i++) { a = Math.min(a, p[i]); b = Math.max(b, p[i]); }
    if (!isFinite(a)) { a = 0; b = 1; }
    const pad = (b - a) * 0.08 || 1;
    if (lo == null) lo = a - pad;
    if (hi == null) hi = b + pad;
  }
  if (hi - lo < 1e-9) hi = lo + 1;
  const X = t => L + (t - t0) / (t1 - t0) * pw, Y = v => T + (1 - (v - lo) / (hi - lo)) * ph;

  // 추정 불가 구간: 회색 빗금
  for (const [a, b] of na) if (b >= t0 && a <= t1) {
    const x0 = X(Math.max(a, t0)), x1 = X(Math.min(b, t1));
    g.fillStyle = 'rgba(135,150,168,.10)'; g.fillRect(x0, T, Math.max(1, x1 - x0), ph);
    g.save(); g.beginPath(); g.rect(x0, T, Math.max(1, x1 - x0), ph); g.clip(); g.strokeStyle = 'rgba(135,150,168,.22)'; g.lineWidth = 1;
    for (let x = x0 - ph; x < x1; x += 7) { g.beginPath(); g.moveTo(x, T + ph); g.lineTo(x + ph, T); g.stroke(); }
    g.restore();
  }
  g.fillStyle = COL.outBg;
  for (const [a, b] of outages) if (b >= t0 && a <= t1) { const x0 = X(Math.max(a, t0)), x1 = X(Math.min(b, t1)); g.fillRect(x0, T, Math.max(1, x1 - x0), ph); }

  const ys = niceStep(hi - lo, 4);
  g.strokeStyle = COL.grid; g.fillStyle = COL.text; g.lineWidth = 1; g.textAlign = 'right'; g.textBaseline = 'middle';
  for (let v = Math.ceil(lo / ys) * ys; v <= hi; v += ys) { const y = Y(v); g.beginPath(); g.moveTo(L, y); g.lineTo(W - Rr, y); g.stroke(); g.fillText(fmt(+v.toFixed(6)), L - 5, y); }
  const span = (t1 - t0) / 1000, xs = niceStep(span, 4);
  g.textAlign = 'center'; g.textBaseline = 'top';
  for (let s = 0; s <= span + 1e-6; s += xs) {
    const x = X(t1 - s * 1000); if (x < L - 1) break;
    g.fillText(s === 0 ? '지금' : s >= 60 ? `-${+(s / 60).toFixed(1)}분` : `-${Math.round(s)}초`, Math.min(Math.max(x, L + 12), W - Rr - 12), H - B + 4);
  }

  g.save(); g.beginPath(); g.rect(L, T, pw, ph); g.clip();
  for (const s of series) {
    const pts = s.pts.filter(inWin), stride = Math.max(1, Math.floor(pts.length / (pw * 1.5)));
    if (s.type === 'band') {
      g.fillStyle = s.color; let seg = [];
      const flush = () => {
        if (seg.length > 1) { g.beginPath(); seg.forEach((p, i) => (i ? g.lineTo(X(p[0]), Y(p[2])) : g.moveTo(X(p[0]), Y(p[2])))); for (let i = seg.length - 1; i >= 0; i--) g.lineTo(X(seg[i][0]), Y(seg[i][1])); g.fill(); }
        seg = [];
      };
      pts.forEach((p, i) => { if (i % stride) return; if (seg.length && p[0] - seg[seg.length - 1][0] > 3000) flush(); seg.push(p); });
      flush();
    } else if (s.type === 'line') {
      g.strokeStyle = s.color; g.lineWidth = s.width || 2; g.setLineDash(s.dash || []); g.beginPath();
      let prev = null;
      pts.forEach((p, i) => {
        if (i % stride && i !== pts.length - 1) return;
        const brk = !prev || p[0] - prev[0] > 3000 || (s.wrap && Math.abs(p[1] - prev[1]) > 180);
        brk ? g.moveTo(X(p[0]), Y(p[1])) : g.lineTo(X(p[0]), Y(p[1])); prev = p;
      });
      g.stroke(); g.setLineDash([]);
    } else {
      g.fillStyle = s.color;
      for (const p of pts) { g.beginPath(); g.arc(X(p[0]), Y(p[1]), s.r || 2.2, 0, 7); g.fill(); }
      if (s.hollow) { g.strokeStyle = s.hollowColor || COL.ref; g.lineWidth = 1.2; for (const p of s.hollow.filter(inWin)) { g.beginPath(); g.arc(X(p[0]), Y(p[1]), 2.6, 0, 7); g.stroke(); } }
    }
  }
  g.restore();
  g.strokeStyle = COL.axis; g.beginPath(); g.moveTo(L, T); g.lineTo(L, T + ph); g.lineTo(W - Rr, T + ph); g.stroke();
}

// 궤적: 지도 타일 없이 미터 단위 평면에 GPS 측정점과 칼만 추정 경로를 그림
export function drawTrack(cv, { ref, gps, kf, cur, follow }) {
  const [g, W, H] = setup(cv);
  if (!ref) {
    g.fillStyle = COL.text; g.textAlign = 'center'; g.font = '13px system-ui,sans-serif';
    g.fillText('시작하거나 시뮬레이션을 켜면 궤적이 그려져요', W / 2, H / 2); return;
  }
  const kx = 111320 * Math.cos(ref.lat * Math.PI / 180), ky = 110540;
  const P = (la, lo) => [(lo - ref.lon) * kx, (la - ref.lat) * ky];
  let cx, cy, span;
  if (follow && cur) {
    [cx, cy] = P(cur.lat, cur.lon);
    span = Math.min(40000, Math.max(250, cur.v * 70, cur.sig * 6));
  } else {
    let a = Infinity, b = -Infinity, c = Infinity, d = -Infinity;
    for (const p of [...kf, ...gps]) { const [x, y] = P(p.lat, p.lon); a = Math.min(a, x); b = Math.max(b, x); c = Math.min(c, y); d = Math.max(d, y); }
    if (!isFinite(a)) { a = b = c = d = 0; }
    cx = (a + b) / 2; cy = (c + d) / 2; span = Math.max(200, (b - a) * 1.15, (d - c) * 1.15 * W / H);
  }
  const s = W / span, X = x => W / 2 + (x - cx) * s, Y = y => H / 2 - (y - cy) * s;

  // 격자 + 축척
  const step = niceStep(span, 5);
  g.strokeStyle = COL.grid; g.lineWidth = 1;
  for (let x = Math.floor((cx - span) / step) * step; x <= cx + span; x += step) { g.beginPath(); g.moveTo(X(x), 0); g.lineTo(X(x), H); g.stroke(); }
  for (let y = Math.floor((cy - span) / step) * step; y <= cy + span; y += step) { g.beginPath(); g.moveTo(0, Y(y)); g.lineTo(W, Y(y)); g.stroke(); }
  const bar = step * s;
  g.strokeStyle = COL.text; g.lineWidth = 2; g.beginPath(); g.moveTo(12, H - 14); g.lineTo(12 + bar, H - 14); g.stroke();
  g.fillStyle = COL.text; g.textAlign = 'left'; g.textBaseline = 'bottom';
  g.fillText(step >= 1000 ? `${step / 1000} km` : `${step} m`, 12, H - 18);
  g.textAlign = 'center'; g.font = '700 12px system-ui,sans-serif'; g.fillText('N', W - 16, 20);
  g.beginPath(); g.moveTo(W - 16, 24); g.lineTo(W - 20, 34); g.lineTo(W - 12, 34); g.closePath(); g.fill();

  // GPS 측정점
  const stride = Math.max(1, Math.floor(gps.length / 3000));
  gps.forEach((p, i) => {
    if (i % stride) return;
    const [x, y] = P(p.lat, p.lon);
    g.beginPath(); g.arc(X(x), Y(y), 2.4, 0, 7);
    if (p.blocked) { g.strokeStyle = COL.ref; g.lineWidth = 1.2; g.stroke(); }
    else if (p.rejected) { g.strokeStyle = COL.err; g.lineWidth = 1.2; g.stroke(); }
    else { g.fillStyle = COL.gps; g.fill(); }
  });

  // 칼만 추정 경로 (GPS 끊김 구간은 주황)
  const ks = Math.max(1, Math.floor(kf.length / 4000));
  g.lineWidth = 2.5;
  for (const out of [false, true]) {
    g.strokeStyle = out ? COL.out : COL.kf; g.beginPath();
    let prev = null;
    kf.forEach((p, i) => {
      if (i % ks && i !== kf.length - 1) return;
      const [x, y] = P(p.lat, p.lon);
      if (prev && p.out === out && p.t - prev.t < 5000) { g.moveTo(X(prev.x), Y(prev.y)); g.lineTo(X(x), Y(y)); }
      prev = { x, y, t: p.t };
    });
    g.stroke();
  }

  // 현재 추정: 불확실도 원(2σ) + 방향 화살표
  if (cur) {
    const [x, y] = P(cur.lat, cur.lon), r = Math.max(4, 2 * cur.sig * s);
    g.fillStyle = cur.out ? 'rgba(255,184,77,.16)' : 'rgba(76,194,255,.16)';
    g.strokeStyle = cur.out ? COL.out : COL.kf; g.lineWidth = 1;
    g.beginPath(); g.arc(X(x), Y(y), Math.min(r, 4 * W), 0, 7); g.fill(); g.stroke();
    g.save(); g.translate(X(x), Y(y));
    if (cur.hdgOk) {
      g.rotate(cur.psi * Math.PI / 180);
      g.fillStyle = '#fff'; g.strokeStyle = cur.out ? COL.out : COL.kf; g.lineWidth = 2;
      g.beginPath(); g.moveTo(0, -12); g.lineTo(8, 8); g.lineTo(0, 3); g.lineTo(-8, 8); g.closePath(); g.fill(); g.stroke();
    } else {   // 방향 추정 불가: 점 + '방향?' 표시
      g.fillStyle = '#fff'; g.strokeStyle = cur.out ? COL.out : COL.kf; g.lineWidth = 2;
      g.beginPath(); g.arc(0, 0, 6, 0, 7); g.fill(); g.stroke();
      g.fillStyle = COL.text; g.font = '12px system-ui,sans-serif'; g.textAlign = 'left'; g.textBaseline = 'middle';
      g.fillText('방향 추정 불가', 11, 0);
    }
    g.restore();
  }
}
