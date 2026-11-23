// GPS + IMU 확장 칼만필터 (EKF)
// 상태 x = [E, N, v, ψ, bg]
//   E, N : 기준점에서 동/북 방향 거리 (m)
//   v    : 속력 (m/s)
//   ψ    : 진행 방향 (rad, 북=0, 시계방향 +)
//   bg   : 자이로 치우침 (rad/s)
// 예측: IMU 입력 u = { a: 진행방향 가속(m/s²), r: 수직축 회전(rad/s), s: 회전 부호, okA, okW }
// 보정: GPS 위치(+ 있으면 속력·방향)

const R = 6371000, TAU = 2 * Math.PI;
export const rad = d => d * Math.PI / 180, deg = r => r * 180 / Math.PI;
export const wrapPi = a => ((a + Math.PI) % TAU + TAU) % TAU - Math.PI;
export const hav = (a, b, c, d) => {
  const p = rad(c - a), q = rad(d - b);
  const h = Math.sin(p / 2) ** 2 + Math.cos(rad(a)) * Math.cos(rad(c)) * Math.sin(q / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
};

// ---- 작은 행렬 연산 ----
export const M = {
  mul: (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0))),
  T: A => A[0].map((_, j) => A.map(r => r[j])),
  add: (A, B) => A.map((r, i) => r.map((v, j) => v + B[i][j])),
  sub: (A, B) => A.map((r, i) => r.map((v, j) => v - B[i][j])),
  I: n => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => +(i === j))),
  diag: d => d.map((v, i) => d.map((_, j) => (i === j ? v : 0))),
  inv(A) {
    const n = A.length, a = A.map((r, i) => [...r, ...M.I(n)[i]]);
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
      [a[c], a[p]] = [a[p], a[c]];
      const d = a[c][c] || 1e-12;
      for (let j = 0; j < 2 * n; j++) a[c][j] /= d;
      for (let r = 0; r < n; r++) if (r !== c) { const f = a[r][c]; if (f) for (let j = 0; j < 2 * n; j++) a[r][j] -= f * a[c][j]; }
    }
    return a.map(r => r.slice(n));
  },
};
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

export class EKF {
  constructor() { this.x = null; this.P = null; this.t = 0; this.lat0 = 0; this.lon0 = 0; this.rej = 0; this.gyroOn = false; }

  toEN(lat, lon) { return [rad(lon - this.lon0) * R * Math.cos(rad(this.lat0)), rad(lat - this.lat0) * R]; }
  toLL(e, n) { return [this.lat0 + deg(n / R), this.lon0 + deg(e / (R * Math.cos(rad(this.lat0))))]; }
  ll() { return this.toLL(this.x[0], this.x[1]); }

  init(f) {
    this.lat0 = f.lat; this.lon0 = f.lon; this.t = f.t; this.rej = 0;
    const moving = f.spd != null && f.spd > 1.5 && f.hdg != null;
    this.x = [0, 0, f.spd ?? 0, moving ? rad(f.hdg) : 0, 0]; this.gyroOn = false; this.last = null;
    this.P = M.diag([f.acc ** 2, f.acc ** 2, f.spd != null ? 1 : 25, moving ? rad(10) ** 2 : 10, rad(0.5) ** 2]);
  }

  // 위치 불확실도 (1σ, m)
  sigma() { return this.x ? Math.sqrt(this.P[0][0] + this.P[1][1]) : Infinity; }

  _step(h, u) {
    // 자이로를 처음 쓰기 시작할 때: 그동안 쌓인 방향 오차를 치우침으로 오해하지 않도록
    // 방향 불확실도를 키우고 방향-치우침 상관을 끊는다
    if (u.okW && !this.gyroOn) {
      this.P[3][3] += rad(15) ** 2;
      for (let i = 0; i < 5; i++) if (i !== 4) { this.P[4][i] = this.P[i][4] = 0; }
    }
    this.gyroOn = u.okW;
    const [e, n, v, p, bg] = this.x, s = Math.sin(p), c = Math.cos(p);
    const a = u.okA ? u.a : 0, w = u.okW ? u.s * u.r - bg : 0;
    this.x = [e + v * s * h, n + v * c * h, Math.max(0, v + a * h), ((p + w * h) % TAU + TAU) % TAU, bg];
    const F = [
      [1, 0, s * h, v * c * h, 0],
      [0, 1, c * h, -v * s * h, 0],
      [0, 0, 1, 0, 0],
      [0, 0, 0, 1, u.okW ? -h : 0],
      [0, 0, 0, 0, 1],
    ];
    // 공정잡음: IMU가 정렬되면 작게(IMU를 믿음), 아니면 크게(속도·방향 일정 가정)
    const qa = u.okA ? 0.3 : 1.0, qw = u.okW ? 0.006 : 0.15;
    const Q = M.diag([0.02 * h, 0.02 * h, qa * qa * h, qw * qw * h, 1e-7 * h]);
    this.P = M.add(M.mul(M.mul(F, this.P), M.T(F)), Q);
  }

  predict(t, u) {
    if (!this.x) return;
    let dt = Math.min((t - this.t) / 1000, 5);
    if (dt <= 0) return;
    while (dt > 1e-6) { const h = Math.min(dt, 0.1); this._step(h, u); dt -= h; }
    this.t = t;
  }

  _update(z, H, Rd, ai, names) {
    const y = z.map((v, i) => v - dot(H[i], this.x));
    if (ai != null) y[ai] = wrapPi(y[ai]);
    const S = M.add(M.mul(M.mul(H, this.P), M.T(H)), M.diag(Rd));
    const K = M.mul(M.mul(this.P, M.T(H)), M.inv(S));
    this.x = this.x.map((v, i) => v + dot(K[i], y));
    this.last = { t: this.t, K, y, names, S };          // 시각화용: 마지막 보정의 게인·잔차
    this.x[2] = Math.max(0, this.x[2]);
    this.x[3] = (this.x[3] % TAU + TAU) % TAU;
    const P = M.mul(M.sub(M.I(5), M.mul(K, H)), this.P);
    this.P = P.map((r, i) => r.map((v, j) => (v + P[j][i]) / 2));
  }

  // GPS 보정. 반환: { d: 예측 위치와 GPS 차이(m), ok: 반영 여부 }
  update(f, u) {
    if (!this.x || this.sigma() > 20000) { this.init(f); return { d: null, ok: true, init: true }; }
    this.predict(f.t, u);
    const [e, n] = this.toEN(f.lat, f.lon);
    const d = Math.hypot(e - this.x[0], n - this.x[1]);
    const gate = 4 * Math.sqrt(this.sigma() ** 2 + f.acc ** 2) + 30;   // 튀는 값 거르기
    if (d > gate && this.rej < 3) { this.rej++; return { d, ok: false }; }
    if (d > gate) { this.init(f); return { d, ok: true, init: true }; }
    this.rej = 0;
    const z = [e, n], H = [[1, 0, 0, 0, 0], [0, 1, 0, 0, 0]], Rd = [f.acc ** 2, f.acc ** 2];
    let ai = null; const names = ['E', 'N'];
    if (f.spd != null) { z.push(f.spd); H.push([0, 0, 1, 0, 0]); Rd.push(0.5 ** 2); names.push('v'); }
    if (f.hdg != null && (f.spd ?? this.x[2]) > 1.5) { z.push(rad(f.hdg)); H.push([0, 0, 0, 1, 0]); Rd.push(rad(5) ** 2); ai = z.length - 1; names.push('ψ'); }
    this._update(z, H, Rd, ai, names);
    if (Math.abs(this.x[0]) > 20000 || Math.abs(this.x[1]) > 20000) {   // 기준점 옮기기 (장거리)
      [this.lat0, this.lon0] = this.ll(); this.x[0] = this.x[1] = 0;
    }
    return { d, ok: true };
  }
}
