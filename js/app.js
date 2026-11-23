// 앱: 센서 → 칼만필터 → 시각화
import { EKF, rad, deg, wrapPi, hav } from './kalman.js';
import { Imu, bindDeviceMotion } from './imu.js';
import { Sim } from './sim.js';
import { timeChart, drawTrack, COL } from './viz.js';

const $ = id => document.getElementById(id);
const OUT_MS = 2500;       // 이 시간 동안 GPS 보정이 없으면 '관성 추정' 구간
const MAX_HIST = 20000;

const kf = new EKF(), imu = new Imu();
let H, ref, prevG, lastFixT, lastRawT, gpsIv, lastKfT, lastImuT;
let mode = null;           // null | 'real' | 'sim'
let watchId = null, sim = null, wake = null, blocked = false, follow = true, winSec = 300;

function resetHist() {
  H = { gps: [], kf: [], err: [], imu: [], gacc: [], gyaw: [], gain: [] };
  ref = null; prevG = null; lastFixT = 0; lastRawT = 0; gpsIv = []; lastKfT = 0; lastImuT = 0;
  kf.x = null; imu.reset();
}
const push = (arr, v) => { arr.push(v); if (arr.length > MAX_HIST) arr.splice(0, arr.length - MAX_HIST); };

/* ---------- GPS 수신 ---------- */
function onGps(f) {
  if (lastRawT) { gpsIv.push((f.t - lastRawT) / 1000); if (gpsIv.length > 10) gpsIv.shift(); }
  lastRawT = f.t;
  if (!ref) ref = { lat: f.lat, lon: f.lon };
  const rec = { t: f.t, lat: f.lat, lon: f.lon, spd: f.spd, hdg: f.hdg, acc: f.acc, blocked };

  if (blocked) {                       // 테스트용 차단: 필터에는 안 넣고 정답 비교에만 사용
    if (kf.x) { kf.predict(f.t, imu.input()); const [la, lo] = kf.ll(); push(H.err, { t: f.t, d: hav(la, lo, f.lat, f.lon), blocked: true }); }
    push(H.gps, rec); prevG = null; imu.clearBin(); return;
  }
  // 정지 판정: GPS 속력≈0 이고 IMU도 조용함(IMU 없으면 GPS만으로)
  const still = f.spd != null && f.spd < 0.3 && (!imu.fresh() || imu.still());
  rec.still = still;
  const res = kf.update(f, imu.input(), still);
  if (!res.ok) { rec.rejected = true; push(H.gps, rec); prevG = null; imu.clearBin(); return; }
  push(H.gps, rec);
  if (res.d != null) push(H.err, { t: f.t, d: res.d, blocked: false });
  if (!res.init && kf.last) {
    const L = kf.last, col = n => L.names.indexOf(n);
    push(H.gain, { t: f.t, kE: L.K[0][col('E')], kv: col('v') >= 0 ? L.K[2][col('v')] : null, kp: col('ψ') >= 0 ? L.K[3][col('ψ')] : null });
  }
  lastFixT = f.t;

  // IMU 정렬 학습: GPS 속력·방향 변화와 IMU 신호를 짝지음
  const sp = f.spd ?? kf.x[2];
  const hd = f.hdg != null && sp > 3 ? f.hdg : kf.x[2] > 3 ? deg(kf.x[3]) : null;
  if (prevG && f.t - prevG.t >= 500 && f.t - prevG.t <= 3000 && f.acc <= 30) {
    const dt = (f.t - prevG.t) / 1000, aG = (sp - prevG.sp) / dt;
    const wG = hd != null && prevG.hd != null ? wrapPi(rad(hd - prevG.hd)) / dt : null;
    imu.learn(aG, wG);
    push(H.gacc, [f.t, aG]); if (wG != null) push(H.gyaw, [f.t, deg(wG)]);
  } else imu.clearBin();
  prevG = { t: f.t, sp, hd };
}

/* ---------- 주기 처리: 예측 + 기록 ---------- */
let imuCnt0 = 0, imuT0 = 0;
setInterval(() => {
  const now = Date.now();
  if (now - imuT0 >= 1000) { imu.hz = imuT0 ? Math.round((imu.cnt - imuCnt0) * 1000 / (now - imuT0)) : 0; imuCnt0 = imu.cnt; imuT0 = now; }
  if (!mode || !kf.x) return;
  const u = imu.input();
  kf.predict(now, u);
  const out = now - lastFixT > OUT_MS;
  if (now - lastKfT >= 500) {
    lastKfT = now;
    const [la, lo] = kf.ll();
    push(H.kf, { t: now, lat: la, lon: lo, v: kf.x[2], psi: deg(kf.x[3]), sv: Math.sqrt(kf.P[2][2]), sp: deg(Math.sqrt(kf.P[3][3])), sig: kf.sigma(), out });
  }
  if (now - lastImuT >= 200 && imu.fresh()) {
    lastImuT = now;
    push(H.imu, { t: now, a: u.okA ? u.a : null, w: u.okW ? deg(u.s * u.r - kf.x[4]) : null,
      h: Math.hypot(...imu.hLP), r: deg(imu.rLP), hz: imu.hz, nA: imu.nA, nW: imu.nW, s: imu.sign(), bg: deg(kf.x[4]) });
  }
}, 100);

/* ---------- 시작/정지 ---------- */
function startReal() {
  stopAll(); resetHist(); mode = 'real';
  bindDeviceMotion(imu, () => mode !== 'real');
  if (!('geolocation' in navigator) || !isSecureContext) { msg('위치 기능은 https 주소에서만 동작해요.'); }
  else watchId = navigator.geolocation.watchPosition(p => {
    const c = p.coords;
    onGps({
      t: Date.now(), lat: c.latitude, lon: c.longitude, acc: Math.max(3, c.accuracy), alt: c.altitude,
      spd: c.speed != null && !isNaN(c.speed) && c.speed >= 0 ? c.speed : null,
      hdg: c.heading != null && !isNaN(c.heading) ? c.heading : null,
    });
  }, e => {
    if (e.code === 1) { msg('위치 권한이 거부됐어요. 설정에서 허용한 뒤 다시 시작하세요. (시뮬레이션은 권한 없이 동작)'); stopAll(); }
  }, { enableHighAccuracy: true, maximumAge: 0, timeout: 60000 });
  keepAwake(true); msg('위성 찾는 중… 창가나 실외에서 더 빨라요.');
}
function startSim() {
  stopAll(); resetHist(); mode = 'sim';
  sim = new Sim(onGps, (a, l, r, dt) => imu.sample(a, l, r, dt));
  sim.start(); window.__sim = sim; window.__kf = kf; window.__imu = imu;
  msg('시뮬레이션 주행 중 — "GPS 차단"을 눌러 관성 추정을 확인해 보세요.');
}
function stopAll() {
  if (watchId != null) navigator.geolocation.clearWatch(watchId);
  if (sim) sim.stop();
  watchId = null; sim = null; mode = null; blocked = false; keepAwake(false);
}
async function keepAwake(on) {
  try { if (on && !wake && navigator.wakeLock) wake = await navigator.wakeLock.request('screen'); if (!on && wake) { await wake.release(); wake = null; } } catch (e) { wake = null; }
}
document.addEventListener('visibilitychange', () => { if (document.visibilityState === 'visible' && mode === 'real') { wake = null; keepAwake(true); } });

$('go').onclick = () => (mode === 'real' ? (stopAll(), msg('정지했어요. 기록은 남아 있어요.')) : startReal());
$('sim').onclick = () => (mode === 'sim' ? (stopAll(), msg('시뮬레이션을 멈췄어요.')) : startSim());
$('blk').onclick = () => { if (!mode) return msg('먼저 시작하거나 시뮬레이션을 켜 주세요.'); blocked = !blocked; msg(blocked ? 'GPS 차단: 필터는 IMU로만 추정하고, 실제 GPS는 회색 점(정답)으로만 표시해요.' : 'GPS 차단 해제'); };
$('csv').onclick = saveCsv;
$('rst').onclick = () => {
  resetHist(); blocked = false;
  msg(mode ? '필터·IMU 정렬·기록을 초기화했어요. 다음 GPS 수신부터 새로 시작해요.' : '필터와 기록을 초기화했어요.');
};
$('fFollow').onclick = () => { follow = true; segOn('fFollow', 'fAll'); };
$('fAll').onclick = () => { follow = false; segOn('fAll', 'fFollow'); };
for (const b of document.querySelectorAll('#win button')) b.onclick = () => { winSec = +b.dataset.w; document.querySelectorAll('#win button').forEach(x => x.classList.toggle('on', x === b)); };
function segOn(a, b) { $(a).classList.add('on'); $(b).classList.remove('on'); }
function msg(t) { $('msg').textContent = t; }

function saveCsv() {
  if (!H.gps.length) return msg('저장할 기록이 없어요.');
  const rows = ['type,t_ms,lat,lon,speed_mps,heading_deg,acc_or_sigma_m,flag,imu_a_est_mps2,imu_yaw_est_dps,imu_h_raw_mps2,imu_yaw_raw_dps,imu_hz,align_a,align_w,yaw_sign,gyro_bias_dps'];
  H.gps.forEach(p => rows.push(['gps', p.t, p.lat, p.lon, p.spd ?? '', p.hdg ?? '', p.acc, p.blocked ? 'blocked' : p.rejected ? 'rejected' : p.still ? 'still' : ''].join(',')));
  H.kf.forEach(p => rows.push(['kf', p.t, p.lat.toFixed(7), p.lon.toFixed(7), p.v.toFixed(3), p.psi.toFixed(2), p.sig.toFixed(2), p.out ? 'imu_only' : ''].join(',')));
  H.imu.forEach(p => rows.push(['imu', p.t, '', '', '', '', '', '', p.a == null ? '' : p.a.toFixed(3), p.w == null ? '' : p.w.toFixed(3),
    p.h.toFixed(3), p.r.toFixed(3), p.hz, p.nA, p.nW, p.s, p.bg.toFixed(4)].join(',')));
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([rows.join('\n')], { type: 'text/csv' }));
  a.download = `kf-log-${new Date().toISOString().slice(0, 16).replace(/[:T]/g, '-')}.csv`;
  document.body.appendChild(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

/* ---------- 화면 ---------- */
const DIRS = ['북', '북북동', '북동', '동북동', '동', '동남동', '남동', '남남동', '남', '남남서', '남서', '서남서', '서', '서북서', '북서', '북북서'];
const fmtM = m => (m < 1000 ? `${Math.round(m)} m` : `${(m / 1000).toFixed(1)} km`);
const fmtT = s => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
function chip(id, cls, text) { $(id).className = 'chip ' + cls; $(id).lastChild.textContent = text; }

function current() {
  if (!kf.x || !mode) return null;
  const [lat, lon] = kf.ll();
  return { lat, lon, v: kf.x[2], psi: deg(kf.x[3]), sig: kf.sigma(), out: Date.now() - lastFixT > OUT_MS };
}

function render() {
  const now = Date.now(), c = current();
  // 상태 칩
  const gapS = (now - lastFixT) / 1000;
  if (!mode) chip('cGps', '', 'GPS 꺼짐');
  else if (blocked) chip('cGps', 'warn', 'GPS 차단(테스트)');
  else if (!lastRawT) chip('cGps', 'warn', 'GPS 찾는 중');
  else if (c && c.out) chip('cGps', 'bad', `GPS 끊김 ${fmtT(gapS)}`);
  else { const last = H.gps[H.gps.length - 1], iv = gpsIv.reduce((a, b) => a + b, 0) / (gpsIv.length || 1); chip('cGps', 'ok', `GPS ±${Math.round(last.acc)} m · ${iv ? (1 / iv).toFixed(1) : '–'} Hz`); }
  if (imu.denied) chip('cImu', 'bad', 'IMU 권한 없음');
  else if (!imu.fresh()) chip('cImu', '', 'IMU 대기');
  else chip('cImu', imu.ready() ? 'ok' : 'warn', imu.ready() ? `IMU ${imu.hz} Hz · 정렬 완료` : `IMU ${imu.hz} Hz · 정렬 ${imu.progress()}/${imu.total}`);
  if (!c) chip('cKf', '', '필터 대기');
  else chip('cKf', c.out ? 'warn' : 'ok', `${c.out ? 'IMU 추정' : 'GPS+IMU'} ±${fmtM(2 * c.sig)}`);

  // 숫자
  const lost = !c || c.sig > 5000;
  $('rV').textContent = lost ? '–' : Math.round(c.v * 3.6);
  $('rVs').textContent = lost ? '' : `±${(2 * Math.sqrt(kf.P[2][2]) * 3.6).toFixed(1)} km/h`;
  const moving = !lost && c.v > 1.5;
  $('rH').textContent = moving ? Math.round(c.psi) : '–';
  $('rHs').textContent = moving ? `${DIRS[Math.round(c.psi / 22.5) % 16]} · ±${Math.round(2 * deg(Math.sqrt(kf.P[3][3])))}°` : lost ? '' : '정지 중';
  $('needle').setAttribute('transform', `rotate(${moving ? c.psi : 0} 20 20)`);
  $('needle').style.opacity = moving ? 1 : 0.25;
  $('rP').innerHTML = lost ? '–' : `${c.lat.toFixed(5)}<br>${c.lon.toFixed(5)}`;
  $('rPs').textContent = lost ? '' : `±${fmtM(2 * c.sig)} (95%)`;
  $('blk').classList.toggle('on', blocked);
  $('go').textContent = mode === 'real' ? '정지' : '시작'; $('go').classList.toggle('stop', mode === 'real');
  $('sim').textContent = mode === 'sim' ? '시뮬레이션 정지' : '시뮬레이션'; $('sim').classList.toggle('on', mode === 'sim');

  // 궤적
  drawTrack($('cvTrack'), { ref, gps: H.gps, kf: H.kf, cur: c, follow });

  // 시계열
  const t1 = mode ? now : (H.kf.length ? H.kf[H.kf.length - 1].t : now);
  const tStart = H.kf.length ? H.kf[0].t : t1 - 60000;
  const t0 = winSec ? t1 - winSec * 1000 : Math.min(tStart, t1 - 30000);
  const outages = []; let o = null;
  for (const p of H.kf) { if (p.out && !o) o = [p.t, p.t]; if (p.out && o) o[1] = p.t; if (!p.out && o) { outages.push(o); o = null; } }
  if (o) outages.push(o);
  const okG = H.gps.filter(p => !p.blocked && !p.rejected), refG = H.gps.filter(p => p.blocked);

  timeChart($('cvV'), { t0, t1, outages, yMin: 0, series: [
    { type: 'band', color: COL.band, pts: H.kf.map(p => [p.t, Math.max(0, (p.v - 2 * p.sv) * 3.6), (p.v + 2 * p.sv) * 3.6]) },
    { type: 'line', color: COL.kf, pts: H.kf.map(p => [p.t, p.v * 3.6]) },
    { type: 'dots', color: COL.gps, pts: okG.filter(p => p.spd != null).map(p => [p.t, p.spd * 3.6]), hollow: refG.filter(p => p.spd != null).map(p => [p.t, p.spd * 3.6]) },
  ] });
  timeChart($('cvH'), { t0, t1, outages, yMin: 0, yMax: 360, series: [
    { type: 'line', color: COL.kf, wrap: true, pts: H.kf.filter(p => p.v > 1.5).map(p => [p.t, p.psi]) },
    { type: 'dots', color: COL.gps, pts: okG.filter(p => p.hdg != null).map(p => [p.t, p.hdg]), hollow: refG.filter(p => p.hdg != null).map(p => [p.t, p.hdg]) },
  ] });
  timeChart($('cvE'), { t0, t1, outages, yMin: 0, fmt: v => (v >= 1000 ? `${v / 1000}k` : v), series: [
    { type: 'line', color: COL.out, dash: [5, 4], width: 1.6, pts: H.kf.map(p => [p.t, 2 * p.sig]) },
    { type: 'dots', color: COL.err, pts: H.err.filter(p => !p.blocked).map(p => [p.t, p.d]), hollow: H.err.filter(p => p.blocked).map(p => [p.t, p.d]), hollowColor: COL.err },
  ] });
  timeChart($('cvK'), { t0, t1, outages, yMin: 0, yMax: 1, series: [
    { type: 'line', color: COL.kf, pts: H.gain.map(p => [p.t, p.kE]) },
    { type: 'line', color: COL.gps, width: 1.6, pts: H.gain.filter(p => p.kv != null).map(p => [p.t, p.kv]) },
    { type: 'line', color: COL.imu, width: 1.6, pts: H.gain.filter(p => p.kp != null).map(p => [p.t, p.kp]) },
  ] });
  timeChart($('cvS'), { t0, t1, outages, yMin: 0, series: [
    { type: 'line', color: COL.kf, pts: H.kf.map(p => [p.t, p.sig]) },
    { type: 'line', color: COL.gps, width: 1.6, pts: H.kf.map(p => [p.t, p.sv * 3.6]) },
    { type: 'line', color: COL.imu, width: 1.6, pts: H.kf.filter(p => p.v > 1.5 && p.sp < 90).map(p => [p.t, p.sp]) },
  ] });
  timeChart($('cvA'), { t0, t1, outages, series: [
    { type: 'line', color: COL.imu, pts: H.imu.filter(p => p.a != null).map(p => [p.t, p.a]) },
    { type: 'dots', color: COL.gps, pts: H.gacc },
  ] });
  timeChart($('cvW'), { t0, t1, outages, series: [
    { type: 'line', color: COL.imu, pts: H.imu.filter(p => p.w != null).map(p => [p.t, p.w]) },
    { type: 'dots', color: COL.gps, pts: H.gyaw },
  ] });

  renderMatrices(now);

  // 차단 중 실제 오차 요약
  const be = H.err.filter(p => p.blocked && p.t > now - 3000).pop();
  $('errNow').textContent = blocked && be ? `차단 중 실제 오차 ${fmtM(be.d)} · 필터 예상 ±${fmtM(2 * c.sig)}` : '';
}

/* ---------- 공분산 P · 칼만 게인 K 표 ---------- */
const ST = ['E', 'N', 'v', 'ψ', 'b<sub>g</sub>'];
const SCALE = [1, 1, 1, 180 / Math.PI, 180 / Math.PI];            // 상태 표시 단위: m, m, m/s, °, °/s
const MSCALE = { E: 1, N: 1, v: 1, 'ψ': 180 / Math.PI };          // 측정 표시 단위: m, m, m/s, °
const UNIT = [' m', ' m', ' m/s', '°', '°/s'], MUNIT = { E: ' m', N: ' m', v: ' m/s', 'ψ': '°' };
function num(x) {
  const a = Math.abs(x);
  return a === 0 ? '0' : a >= 100 ? x.toFixed(0) : a >= 1 ? x.toFixed(2) : a >= 0.01 ? x.toFixed(3) : x.toExponential(1);
}
const pos = a => `rgba(76,194,255,${(0.08 + 0.62 * Math.min(1, a)).toFixed(2)})`;
const neg = a => `rgba(255,107,107,${(0.08 + 0.62 * Math.min(1, a)).toFixed(2)})`;
let matKey = '';
function renderMatrices(now) {
  const key = kf.x ? `${kf.t}|${kf.last ? kf.last.t : 0}` : 'none';
  if (key === matKey && kf.x) { if (kf.last) $('kAge').textContent = ageText(now); return; }
  matKey = key;
  const head = '<tr><th></th>' + ST.map((s, i) => `<th>${s}<br><small style="font-weight:400">${UNIT[i].trim()}</small></th>`).join('') + '</tr>';
  if (!kf.x) {
    $('tP').innerHTML = head + ST.map(s => `<tr><th>${s}</th>${ST.map(() => '<td>–</td>').join('')}</tr>`).join('');
    $('tK').innerHTML = '<tr><th></th><th>E</th><th>N</th><th>v</th><th>ψ</th></tr>' + ST.map(s => `<tr><th>${s}</th><td>–</td><td>–</td><td>–</td><td>–</td></tr>`).join('');
    $('kAge').textContent = '필터 대기 중'; return;
  }
  const P = kf.P, sd = P.map((r, i) => Math.sqrt(Math.max(0, r[i])));
  $('tP').innerHTML = head + ST.map((s, i) => `<tr><th>${s}</th>${ST.map((_, j) => {
    if (i === j) return `<td class="dg" title="σ (${UNIT[i].trim()})">${num(sd[i] * SCALE[i])}</td>`;
    const rho = P[i][j] / (sd[i] * sd[j] || 1);
    return `<td style="background:${rho >= 0 ? pos(Math.abs(rho)) : neg(Math.abs(rho))}" title="ρ">${rho.toFixed(2)}</td>`;
  }).join('')}</tr>`).join('');
  const L = kf.last;
  if (!L) { $('tK').innerHTML = '<tr><td>아직 보정 없음</td></tr>'; $('kAge').textContent = ''; return; }
  $('tK').innerHTML = '<tr><th></th>' + L.names.map(n => `<th>${n}</th>`).join('') + '</tr>' +
    ST.map((s, i) => `<tr><th>${s}</th>${L.names.map((n, j) => {
      const v = L.K[i][j] * SCALE[i] / MSCALE[n];
      return `<td style="background:${v >= 0 ? pos(Math.abs(v)) : neg(Math.abs(v))}">${num(v)}</td>`;
    }).join('')}</tr>`).join('') +
    `<tr class="y"><th>y</th>${L.names.map((n, j) => `<td>${L.y[j] >= 0 ? '+' : ''}${num(L.y[j] * MSCALE[n])}${MUNIT[n]}</td>`).join('')}</tr>`;
  $('kAge').textContent = ageText(now);
}
function ageText(now) {
  const s = (now - kf.last.t) / 1000;
  return s < 3 ? `마지막 GPS 보정 ${s.toFixed(1)}초 전` : `GPS 보정 없음 ${fmtT(s)} · 예측만 진행 중`;
}

setInterval(render, 250);
new ResizeObserver(render).observe(document.body);

resetHist();
render();
if ('serviceWorker' in navigator) navigator.serviceWorker.register('sw.js').catch(() => {});
