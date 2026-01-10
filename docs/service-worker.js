const CACHE_NAME = 'snek-trackr-v7';
const ASSETS = [
  './',
  './index.html',
  './manifest.json'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(ASSETS))
  );
});

self.addEventListener('fetch', (event) => {
  // For API requests, always go to network
  if (event.request.url.includes('blynk.cloud') || event.request.url.includes('influxdata.com')) {
    event.respondWith(fetch(event.request));
    return;
  }

  // For static assets, try cache first, then network
  event.respondWith(
    caches.match(event.request).then((response) => {
      return response || fetch(event.request);
    })
  );
});
