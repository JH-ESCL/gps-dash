const C = 'kf-viz-1.11.0';   // js/version.js 와 맞춤
const CORE = ['./', './index.html', './js/app.js', './js/kalman.js', './js/imu.js', './js/sim.js', './js/viz.js', './js/version.js', './js/basemap.js', './js/compass.js'];
const EXTRA = ['./manifest.webmanifest', './icon.svg', './icon-192.png', './icon-512.png'];

self.addEventListener('install', e => {
  e.waitUntil(caches.open(C).then(async c => {
    await c.addAll(CORE);
    await Promise.all(EXTRA.map(u => c.add(u).catch(() => {})));
  }));
  self.skipWaiting();
});

self.addEventListener('activate', e => {
  e.waitUntil(caches.keys().then(keys => Promise.all(keys.filter(k => k !== C && k !== TILES).map(k => caches.delete(k)))));
  self.clients.claim();
});

// 지도 타일: 캐시 우선, 받은 타일은 저장(오프라인에서도 보이게), 최대 3000장
const TILES = 'kf-tiles';
let puts = 0;
async function tileFetch(req) {
  const c = await caches.open(TILES), hit = await c.match(req);
  if (hit) return hit;
  const r = await fetch(req);
  if (r.ok) {
    await c.put(req, r.clone());
    if (++puts % 100 === 0) { const ks = await c.keys(); for (const k of ks.slice(0, Math.max(0, ks.length - 3000))) await c.delete(k); }
  }
  return r;
}

// 앱 파일: 네트워크 우선(최신 버전), 실패하면 캐시 → 오프라인에서도 열림
self.addEventListener('fetch', e => {
  if (e.request.method !== 'GET') return;
  if (new URL(e.request.url).hostname === 'tile.openstreetmap.org') { e.respondWith(tileFetch(e.request)); return; }
  e.respondWith(
    fetch(e.request).then(r => {
      if (r.ok && new URL(e.request.url).origin === location.origin) { const cp = r.clone(); caches.open(C).then(c => c.put(e.request, cp)); }
      return r;
    }).catch(() => caches.match(e.request, { ignoreSearch: true }).then(r => r || caches.match('./index.html')))
  );
});
