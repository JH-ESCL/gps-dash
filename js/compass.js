// 나침반(자기센서) — 폰이 향한 방향(자북 기준)
// 진행 방향과는 다르므로, 위성 GPS 진행 방향과의 차이(오프셋)를 학습한 뒤에만 필터에 넣는다.
// 오프셋에는 폰 거치 각도와 자북·진북 차이(편각)가 함께 들어간다.
const D = Math.PI / 180;
const wrap180 = a => ((a + 540) % 360) - 180;

export class Compass {
  constructor() { this.reset(); this.on = false; this.denied = false; this.last = 0; this.src = ''; }

  reset(note = '') {
    Object.assign(this, { hdg: null, axis: null, S: 0, C: 0, W: 0, n: 0, offset: null, ostd: null, note, disturbedUntil: 0, hist: [] });
  }

  fresh() { return this.on && Date.now() - this.last < 1500; }
  aligned() { return this.offset != null && this.n >= 10 && this.ostd < 20; }

  // 안드로이드 크롬: deviceorientationabsolute (alpha·beta·gamma, 지구 좌표 기준)
  fromEuler(a, b, g) {
    // R = Rz(α)·Rx(β)·Ry(γ), 지구 좌표 x=동, y=북, z=위
    const ca = Math.cos(a * D), sa = Math.sin(a * D), cb = Math.cos(b * D), sb = Math.sin(b * D), cg = Math.cos(g * D), sg = Math.sin(g * D);
    const yAx = [-sa * cb, ca * cb, sb];                                         // 폰 윗변 방향
    const zAx = [ca * sg + sa * sb * cg, sa * sg - ca * sb * cg, cb * cg];          // 화면 바깥 방향
    const back = zAx.map(v => -v);                                                // 후면 카메라 방향
    const h = v => Math.hypot(v[0], v[1]);
    // 수평 성분이 큰 축을 고르고, 그 축이 거의 수직이 되면 바꿈(바꾸면 오프셋 다시 학습)
    if (!this.axis || (this.axis === 'y' ? h(yAx) : h(back)) < 0.3) {
      const ax = h(yAx) >= h(back) ? 'y' : 'back';
      if (this.axis && ax !== this.axis) this.reset('폰 자세가 바뀌어 나침반 재정렬');
      this.axis = ax;
    }
    const v = this.axis === 'y' ? yAx : back;
    if (h(v) < 0.2) return;
    this.push((Math.atan2(v[0], v[1]) / D + 360) % 360);
  }

  // iOS: webkitCompassHeading (자북 기준, 시계방향)
  fromIOS(hdg) { this.axis = 'ios'; this.push(hdg); }

  push(h) {
    this.on = true; this.last = Date.now();
    // 원형 저역통과 (0.3초)
    if (this.hdg == null) this.hdg = h;
    else { const k = 0.25; this.hdg = (this.hdg + k * wrap180(h - this.hdg) + 360) % 360; }
    this.hist.push([this.last, this.hdg]); while (this.hist.length && this.hist[0][0] < this.last - 3000) this.hist.shift();
  }

  // 위성 GPS 진행 방향으로 오프셋 학습 (원형 평균, 지수 망각)
  learn(gpsHdg) {
    if (!this.fresh() || this.hdg == null) return;
    const d = wrap180(gpsHdg - this.hdg) * D, lam = 0.97;
    this.S = lam * this.S + Math.sin(d); this.C = lam * this.C + Math.cos(d); this.W = lam * this.W + 1; this.n++;
    const Rbar = Math.hypot(this.S, this.C) / this.W;
    this.offset = Math.atan2(this.S, this.C) / D;
    this.ostd = Math.sqrt(-2 * Math.log(Math.max(1e-6, Math.min(1, Rbar)))) / D;   // 원형 표준편차(°)
  }

  // 자이로와 비교해 자기 교란 감지: 2초 동안 나침반 변화가 자이로 회전과 12° 넘게 다르면 5초간 사용 중지
  checkDisturb(gyroDeltaDeg) {
    if (this.hist.length < 2 || gyroDeltaDeg == null) return;
    const now = this.last, old = this.hist.find(p => p[0] >= now - 2000);
    if (!old || now - old[0] < 1500) return;
    const dc = wrap180(this.hdg - old[1]);
    if (Math.abs(dc - gyroDeltaDeg) > 12) this.disturbedUntil = now + 5000;
  }
  disturbed() { return Date.now() < this.disturbedUntil; }

  // 필터에 넣을 진행 방향 측정 (°)과 표준편차(°)
  // gyroOk: 자이로가 보정돼 있으면 나침반은 장시간 흐름만 막도록 신뢰도를 크게 낮춤
  measurement(gyroOk = false) {
    if (!this.fresh() || !this.aligned() || this.disturbed()) return null;
    return { hdg: (this.hdg + this.offset + 360) % 360, sd: gyroOk ? Math.max(40, 20 + this.ostd) : Math.max(12, 8 + this.ostd) };
  }
}

export function bindCompass(cp, isPaused) {
  if (typeof DeviceOrientationEvent === 'undefined') { cp.denied = true; return; }
  const onAbs = e => { if (isPaused() || e.alpha == null) return; cp.src = 'absolute'; cp.fromEuler(e.alpha, e.beta, e.gamma); };
  const onRel = e => {
    if (isPaused()) return;
    if (e.webkitCompassHeading != null && !isNaN(e.webkitCompassHeading)) { cp.src = 'ios'; cp.fromIOS(e.webkitCompassHeading); }
    else if (e.absolute && e.alpha != null && cp.src !== 'absolute') { cp.src = 'absolute'; cp.fromEuler(e.alpha, e.beta, e.gamma); }
  };
  const bind = () => {
    if (bindCompass.bound) return; bindCompass.bound = true;
    if ('ondeviceorientationabsolute' in window) addEventListener('deviceorientationabsolute', onAbs);
    addEventListener('deviceorientation', onRel);
  };
  if (typeof DeviceOrientationEvent.requestPermission === 'function')
    DeviceOrientationEvent.requestPermission().then(p => (p === 'granted' ? bind() : (cp.denied = true))).catch(() => { cp.denied = true; });
  else bind();
}
