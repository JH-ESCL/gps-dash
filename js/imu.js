// IMU 전처리 + 자동 정렬
// 폰이 어떤 자세로 놓여 있어도 쓸 수 있도록:
//  1) 중력 방향(up)을 저역통과로 구하고
//  2) 수평 가속 h(폰 좌표) 와 수직축 회전 r = ω·up 을 뽑은 뒤
//  3) GPS가 잡히는 동안 "진행방향 가속 = f·[h,1]" 을 재귀 최소제곱으로, 회전 부호(s)를 상관으로 학습한다.
import { M } from './kalman.js';

const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const NEED_A = 8, NEED_W = 6;
const loadCal = k => { try { const v = JSON.parse(localStorage.getItem(k)); return v && (v.s === 1 || v.s === -1) ? v : null; } catch (e) { return null; } };

export class Imu {
  constructor(store = 'imu.cal') {
    this.on = false; this.last = 0; this.cnt = 0; this.hz = 0; this.denied = false; this.store = store;
    this.saved = loadCal(store); this.reset();
  }

  // ---- 저장된 자이로 보정 (회전 부호·치우침·자이로 좌표 부호): 폰 자세와 무관해 다음 실행에도 그대로 씀 ----
  saveCal(bg) {
    const s = this.sign(); if (!s || this.nW < NEED_W) return false;
    this.saved = { s, bg, gs: this.ge && this.ge[0] > this.ge[1] ? -1 : 1, at: Date.now() };
    try { localStorage.setItem(this.store, JSON.stringify(this.saved)); } catch (e) {}
    return true;
  }
  // 저장 위치 전환 (실제 센서 / 시뮬레이션 따로)
  useStore(key) { this.store = key; this.saved = loadCal(key); }
  clearCal() { this.saved = null; try { localStorage.removeItem(this.store); } catch (e) {} }

  reset(note = '') {
    Object.assign(this, {
      g: null, gc: null, ge: null, up: null, up0: null, hLP: [0, 0, 0], rLP: 0,
      bin: { h: [0, 0, 0], r: 0, n: 0 },
      A: M.diag([0.05, 0.05, 0.05, 0.05]), b: [0, 0, 0, 0], f: null, nA: 0,
      Srr: 0, Srp: 0, Spp: 0, nW: 0, note, tilt: 0,
    });
  }

  // aig: 중력 포함 가속, lin: 중력 제외 가속(없으면 null), rot: 각속도(rad/s), 모두 폰 좌표
  sample(aig, lin, rot, dt) {
    this.on = true; this.last = Date.now(); this.cnt++;
    // 원시값 누적(기록용): 중력 포함 가속 3축 + 자이로 3축 평균
    const R6 = this.raw || (this.raw = { s: [0, 0, 0, 0, 0, 0], n: 0 });
    aig.concat(rot).forEach((v, i) => { R6.s[i] += v; }); R6.n++;
    const k1 = Math.min(1, dt / 0.3);
    this.aLP = this.aLP ? this.aLP.map((v, i) => v + k1 * (aig[i] - v)) : aig.slice();
    // 중력 방향: 상보 필터 — 자이로로 폰 회전을 따라가고(g ← g + (g×ω)dt), 가속도계로 천천히 보정.
    // 손에 든 폰처럼 자세가 계속 바뀌어도 '위쪽'을 놓치지 않음.
    // 기기마다 자이로 좌표 부호가 다를 수 있어 +ω, −ω 두 후보를 함께 돌리고 가속도계와 더 잘 맞는 쪽을 씀.
    if (!this.gc) { this.gc = [aig.slice(), aig.slice()]; this.ge = this.saved && this.saved.gs === -1 ? [0.3, 0] : [0, this.saved ? 0.3 : 0]; }
    else {
      this.gc = this.gc.map((g, c) => {
        const sg = c ? -1 : 1, [gx, gy, gz] = g, [wx, wy, wz] = rot.map(v => v * sg * dt);
        let n = [gx + (gy * wz - gz * wy), gy + (gz * wx - gx * wz), gz + (gx * wy - gy * wx)];
        const d = Math.hypot(...this.aLP.map((v, i) => v - n[i]));
        const k = Math.min(1, dt / (d < 0.5 ? 1.5 : 15));      // 차량 가속(원심·가감속) 중엔 아주 천천히만 보정
        n = n.map((v, i) => v + k * (this.aLP[i] - v));
        this.ge[c] += Math.min(1, dt / 10) * (d - this.ge[c]);
        return n;
      });
    }
    this.g = this.gc[this.ge[0] <= this.ge[1] ? 0 : 1];
    const gm = Math.hypot(...this.g) || 1, up = this.g.map(v => v / gm);
    this.up = up;
    const a = lin || aig.map((v, i) => v - this.g[i]);
    const av = dot(a, up), h = a.map((v, i) => v - av * up[i]);
    const r = dot(rot, up);                                    // 수직축 회전: 폰 자세와 무관
    const al = Math.min(1, dt / 0.3), ar = Math.min(1, dt / 0.1);
    this.hLP = this.hLP.map((v, i) => v + al * (h[i] - v));
    this.rLP += ar * (r - this.rLP);
    this.bin.h = this.bin.h.map((v, i) => v + h[i]); this.bin.r += r; this.bin.n++;
    // 폰 자세가 30° 넘게 바뀐 채 3초 지나면 '진행방향 가속' 정렬만 다시 (회전 정렬은 자세와 무관해 유지)
    if (this.up0 && dot(up, this.up0) < Math.cos(30 * Math.PI / 180)) {
      this.tilt += dt;
      if (this.tilt > 3) { this.resetAccel(); this.note = '폰 자세가 바뀌어 가속 재정렬'; }
    } else this.tilt = 0;
  }

  resetAccel() { Object.assign(this, { A: M.diag([0.05, 0.05, 0.05, 0.05]), b: [0, 0, 0, 0], f: null, nA: 0, up0: null, tilt: 0 }); }


  // 기록용 원시값 평균을 꺼내고 비움
  takeRaw() { const R6 = this.raw; if (!R6 || !R6.n) return null; const m = R6.s.map(v => v / R6.n); this.raw = { s: [0, 0, 0, 0, 0, 0], n: 0 }; return m; }

  clearBin() { this.bin = { h: [0, 0, 0], r: 0, n: 0 }; }

  // GPS 두 시점 사이: aG = 속력 변화율, wG = 방향 변화율 (없으면 null)
  learn(aG, wG) {
    const { n } = this.bin, h = [...this.bin.h.map(v => v / (n || 1)), 1], r = this.bin.r / (n || 1);
    this.clearBin();
    if (n < 3 || !this.up) return;
    if (!this.up0) this.up0 = this.up.slice();
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
  // 정지 판정 보조: 수평 가속과 회전이 거의 없음
  still() { return this.fresh() && Math.hypot(...this.hLP) < 0.35 && Math.abs(this.rLP) < 0.03; }
  learnedSign() { const c = this.corr(); return Math.abs(c) > 0.6 ? Math.sign(c) : 0; }
  // 이번 실행에서 확정한 부호가 있으면 그것, 없으면 저장된 부호
  sign() { return this.learnedSign() || (this.saved ? this.saved.s : 0); }
  gyroReady() { return (this.nW >= NEED_W && !!this.learnedSign()) || !!this.saved; }
  fresh() { return this.on && Date.now() - this.last < 2000; }
  ready() { return this.nA >= NEED_A && this.gyroReady(); }
  progress() { return Math.min(this.nA, NEED_A) + (this.gyroReady() ? NEED_W : Math.min(this.nW, NEED_W)); }
  get total() { return NEED_A + NEED_W; }

  // 칼만 예측에 넣을 입력
  input() {
    const fr = this.fresh(), s = this.sign();
    const okA = fr && this.nA >= NEED_A && !!this.f, okW = fr && this.gyroReady() && s !== 0;
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
