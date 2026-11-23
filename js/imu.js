// IMU 전처리 + 자동 정렬
// 폰이 어떤 자세로 놓여 있어도 쓸 수 있도록:
//  1) 중력 방향(up)을 저역통과로 구하고
//  2) 수평 가속 h(폰 좌표) 와 수직축 회전 r = ω·up 을 뽑은 뒤
//  3) GPS가 잡히는 동안 "진행방향 가속 = f·[h,1]" 을 재귀 최소제곱으로, 회전 부호(s)를 상관으로 학습한다.
import { M } from './kalman.js';

const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const NEED_A = 8, NEED_W = 6;

export class Imu {
  constructor() { this.on = false; this.last = 0; this.cnt = 0; this.hz = 0; this.denied = false; this.reset(); }

  reset(note = '') {
    Object.assign(this, {
      g: null, up: null, hLP: [0, 0, 0], rLP: 0,
      bin: { h: [0, 0, 0], r: 0, n: 0 },
      A: M.diag([0.05, 0.05, 0.05, 0.05]), b: [0, 0, 0, 0], f: null, nA: 0,
      Srr: 0, Srp: 0, Spp: 0, nW: 0, note, tilt: 0,
    });
  }

  // aig: 중력 포함 가속, lin: 중력 제외 가속(없으면 null), rot: 각속도(rad/s), 모두 폰 좌표
  sample(aig, lin, rot, dt) {
    this.on = true; this.last = Date.now(); this.cnt++;
    // 중력 방향: 짧게 평균한 가속이 ≈9.8이고 회전이 거의 없을 때만 천천히 갱신
    // → 커브의 원심가속, 가감속에 기울지 않음
    const k1 = Math.min(1, dt / 0.3);
    this.aLP = this.aLP ? this.aLP.map((v, i) => v + k1 * (aig[i] - v)) : aig.slice();
    this.wLP = (this.wLP ?? 0) + k1 * (Math.hypot(...rot) - (this.wLP ?? 0));
    if (!this.g) this.g = aig.slice();
    const diff = Math.hypot(...this.aLP.map((v, i) => v - this.g[i]));
    if (diff < 0.35 && this.wLP < 0.03) { const k = Math.min(1, dt / 5); this.g = this.g.map((v, i) => v + k * (this.aLP[i] - v)); }
    const gm = Math.hypot(...this.g) || 1, up = this.g.map(v => v / gm);
    this.up = up;
    const a = lin || aig.map((v, i) => v - this.g[i]);
    const av = dot(a, up), h = a.map((v, i) => v - av * up[i]);
    const r = dot(rot, up);
    const al = Math.min(1, dt / 0.3), ar = Math.min(1, dt / 0.1);
    this.hLP = this.hLP.map((v, i) => v + al * (h[i] - v));
    this.rLP += ar * (r - this.rLP);
    this.bin.h = this.bin.h.map((v, i) => v + h[i]); this.bin.r += r; this.bin.n++;
    // 폰 자세 변화 감지: 짧은 평균 가속이 중력 방향과 30° 넘게 어긋난 상태가 3초 지속되면 재정렬
    const cosA = dot(this.aLP, this.g) / ((Math.hypot(...this.aLP) || 1) * gm);
    if (cosA < Math.cos(30 * Math.PI / 180)) { this.tilt += dt; if (this.tilt > 3) { this.reset('폰 자세가 바뀌어 재정렬'); return; } }
    else this.tilt = 0;
  }

  clearBin() { this.bin = { h: [0, 0, 0], r: 0, n: 0 }; }

  // GPS 두 시점 사이: aG = 속력 변화율, wG = 방향 변화율 (없으면 null)
  learn(aG, wG) {
    const { n } = this.bin, h = [...this.bin.h.map(v => v / (n || 1)), 1], r = this.bin.r / (n || 1);
    this.clearBin();
    if (n < 3 || !this.up) return;
    const lam = 0.997;
    if (aG != null && Math.abs(aG) < 4) {
      this.A = this.A.map((row, i) => row.map((v, j) => lam * v + h[i] * h[j]));
      this.b = this.b.map((v, i) => lam * v + h[i] * aG);
      if (Math.abs(aG) > 0.4) this.nA++;                               // 실제 가감속만 학습 근거로 셈
      this.f = M.inv(this.A).map(row => dot(row, this.b));
    }
    if (wG != null && Math.abs(wG) < 0.5) {
      this.Srr = lam * this.Srr + r * r; this.Srp = lam * this.Srp + r * wG; this.Spp = lam * this.Spp + wG * wG;
      if (Math.abs(wG) > 0.03 && Math.abs(r) > 0.03) this.nW++;      // 실제 커브(>1.7°/s)만 학습 근거로 셈
    }
  }

  // 자이로와 GPS 방향 변화의 상관이 뚜렷할 때만 부호 확정
  corr() { return this.Srr > 1e-4 && this.Spp > 1e-4 ? this.Srp / Math.sqrt(this.Srr * this.Spp) : 0; }
  sign() { const c = this.corr(); return Math.abs(c) > 0.6 ? Math.sign(c) : 0; }
  fresh() { return this.on && Date.now() - this.last < 2000; }
  ready() { return this.nA >= NEED_A && this.nW >= NEED_W; }
  progress() { return Math.min(this.nA, NEED_A) + Math.min(this.nW, NEED_W); }
  get total() { return NEED_A + NEED_W; }

  // 칼만 예측에 넣을 입력
  input() {
    const fr = this.fresh(), s = this.sign();
    const okA = fr && this.nA >= NEED_A && !!this.f, okW = fr && this.nW >= NEED_W && s !== 0;
    return { a: okA ? clamp(dot(this.f, [...this.hLP, 1]), -5, 5) : 0, r: this.rLP, s, okA, okW };
  }
}

// 브라우저 DeviceMotion 연결
export function bindDeviceMotion(imu, isPaused) {
  if (typeof DeviceMotionEvent === 'undefined') { imu.denied = true; return; }
  const on = e => {
    if (isPaused()) return;
    const g = e.accelerationIncludingGravity; if (!g || g.x == null) return;
    const r = e.rotationRate, l = e.acceleration, iv = e.interval || 16;
    const dt = clamp(iv > 1 ? iv / 1000 : iv, 0.005, 0.2);
    const d2r = Math.PI / 180;
    imu.sample([g.x, g.y, g.z], l && l.x != null ? [l.x, l.y, l.z] : null,
      r && r.alpha != null ? [r.beta * d2r, r.gamma * d2r, r.alpha * d2r] : [0, 0, 0], dt);
  };
  const bind = () => { if (!bindDeviceMotion.bound) { addEventListener('devicemotion', on); bindDeviceMotion.bound = true; } };
  if (typeof DeviceMotionEvent.requestPermission === 'function')
    DeviceMotionEvent.requestPermission().then(p => (p === 'granted' ? bind() : (imu.denied = true))).catch(() => { imu.denied = true; });
  else bind();
}
