// 시뮬레이션: 비스듬히 놓인 폰을 실은 차량의 GPS(1 Hz) + IMU(50 Hz)를 만들어 낸다.
// 실제 센서 없이 필터와 시각화를 확인하는 용도.
import { M, rad, deg } from './kalman.js';

const R = 6371000;
const gauss = s => (Math.random() + Math.random() + Math.random() + Math.random() - 2) * s * 1.22;
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

// [지속(s), 가속(m/s²), 회전(°/s, 시계방향 +)]
const ROUTE = [
  [4, 0, 0], [10, 1.5, 0], [10, 0, 0], [4, -1.5, 0], [6, 0, 15], [8, 1.5, 0], [15, 0, 0],
  [12, 0, 3], [6, -2.0, 0], [5, 0, -18], [10, 1.2, 0], [10, 0.8, 0], [20, 0, 1.5],
  [10, -2.0, 0], [6, 0, 15], [6, -1.5, 0], [5, 0, 0],
];
const LOOP = ROUTE.reduce((s, p) => s + p[0], 0);

export class Sim {
  constructor(onGps, onImu, opt = {}) { this.onGps = onGps; this.onImu = onImu; this.timer = null; this.hand = !!opt.hand; this.coarse = !!opt.coarse; }

  start() {
    this.T = { lat: 37.5547, lon: 126.9707, v: 0, psi: rad(35) };
    this.t0 = this.prev = Date.now(); this.nextFix = this.t0;
    // 폰 자세: 차량 좌표(오른쪽, 앞, 위) → 폰 좌표 (기울기 55°, 비틀림 35°)
    const ax = rad(55), az = rad(35);
    const Rx = [[1, 0, 0], [0, Math.cos(ax), -Math.sin(ax)], [0, Math.sin(ax), Math.cos(ax)]];
    const Rz = [[Math.cos(az), -Math.sin(az), 0], [Math.sin(az), Math.cos(az), 0], [0, 0, 1]];
    this.Rd = M.mul(Rx, Rz);
    this.gyroBias = 0.004;   // rad/s
    this.timer = setInterval(() => this.tick(), 20);
  }

  stop() { clearInterval(this.timer); this.timer = null; }

  tick() {
    const now = Date.now(), dt = Math.min((now - this.prev) / 1000, 0.1); this.prev = now;
    let el = ((now - this.t0) / 1000) % LOOP, a = 0, w = 0;
    for (const [d, pa, pw] of ROUTE) { if (el < d) { a = pa; w = rad(pw); break; } el -= d; }
    const T = this.T;
    T.v = Math.max(0, Math.min(33, T.v + a * dt));
    if (T.v === 0 || T.v === 33) a = 0;
    if (T.v < 0.5) w = 0;
    T.psi += w * dt;
    const dd = T.v * dt / R, p1 = rad(T.lat), l1 = rad(T.lon);
    const p2 = Math.asin(Math.sin(p1) * Math.cos(dd) + Math.cos(p1) * Math.sin(dd) * Math.cos(T.psi));
    T.lon = deg(l1 + Math.atan2(Math.sin(T.psi) * Math.sin(dd) * Math.cos(p1), Math.cos(dd) - Math.sin(p1) * Math.sin(p2)));
    T.lat = deg(p2);

    // IMU (차량 좌표 → 폰 좌표)
    // 손에 든 폰: 폰이 자기 x축으로 ±25° 정도 계속 흔들림 (자이로에 손 회전이 더해짐)
    const ts = (now - this.t0) / 1000;
    let C = this.Rd, wHand = [0, 0, 0];
    if (this.hand) {
      const th = 0.3 * Math.sin(0.9 * ts) + 0.15 * Math.sin(2.3 * ts + 1), thd = 0.27 * Math.cos(0.9 * ts) + 0.345 * Math.cos(2.3 * ts + 1);
      const c = Math.cos(th), s = Math.sin(th);
      C = M.mul([[1, 0, 0], [0, c, -s], [0, s, c]], this.Rd); wHand = [-thd, 0, 0];
    }
    const toDev = v => C.map(r => dot(r, v));
    const lin = [T.v * w + gauss(0.3), a + gauss(0.3), gauss(0.3)];
    const gyro = toDev([gauss(0.01), gauss(0.01), -w + this.gyroBias + gauss(0.005)]).map((v, i) => v + wHand[i]);
    // 브라우저 원시 형식(표준)으로: [alpha=z, beta=x, gamma=y]
    this.onImu(toDev([lin[0], lin[1], lin[2] + 9.81]), toDev(lin), [gyro[2], gyro[0], gyro[1]], dt);

    // GPS 1 Hz
    if (now >= this.nextFix && this.coarse) {          // 위성 GPS 없음: Wi‑Fi·기지국 대략 위치 8초마다
      this.nextFix += 8000;
      const n = 120 / 111320;
      this.onGps({ t: now, lat: T.lat + gauss(n), lon: T.lon + gauss(n / Math.cos(p2)), acc: 150, alt: null, spd: null, hdg: null });
    } else if (now >= this.nextFix) {
      this.nextFix += 1000;
      const n = 3.5 / 111320;
      this.onGps({
        t: now, lat: T.lat + gauss(n), lon: T.lon + gauss(n / Math.cos(p2)), acc: 4, alt: 30,
        spd: Math.max(0, T.v + gauss(0.15)), hdg: T.v > 1 ? (deg(T.psi) + gauss(1.5) + 360) % 360 : null,
      });
    }
  }
}
