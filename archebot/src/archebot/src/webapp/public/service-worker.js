// Service worker for offline map functionality

const CACHE_NAME = "map-tiles-cache-v1"
const TILE_CACHE_NAME = "map-tiles"

// Files to cache immediately on install
const PRECACHE_URLS = [
  "/",
  "/index.html",
  "/images/marker-icon.png",
  "/images/marker-icon-2x.png",
  "/images/marker-shadow.png",
  "/images/error-tile.png",
]

// Install event - precache essential files
self.addEventListener("install", (event) => {
  event.waitUntil(
    caches
      .open(CACHE_NAME)
      .then((cache) => cache.addAll(PRECACHE_URLS))
      .then(() => self.skipWaiting()),
  )
})

// Activate event - clean up old caches
self.addEventListener("activate", (event) => {
  const currentCaches = [CACHE_NAME, TILE_CACHE_NAME]
  event.waitUntil(
    caches
      .keys()
      .then((cacheNames) => {
        return cacheNames.filter((cacheName) => !currentCaches.includes(cacheName))
      })
      .then((cachesToDelete) => {
        return Promise.all(
          cachesToDelete.map((cacheToDelete) => {
            return caches.delete(cacheToDelete)
          }),
        )
      })
      .then(() => self.clients.claim()),
  )
})

// Fetch event - serve from cache or network
self.addEventListener("fetch", (event) => {
  // Handle map tile requests specially
  if (event.request.url.includes("/tiles/")) {
    event.respondWith(handleTileRequest(event.request))
    return
  }

  // For other requests, try the cache first, then network
  event.respondWith(
    caches.match(event.request).then((cachedResponse) => {
      if (cachedResponse) {
        return cachedResponse
      }
      return fetch(event.request).then((response) => {
        // Don't cache non-successful responses
        if (!response || response.status !== 200 || response.type !== "basic") {
          return response
        }

        // Clone the response since it can only be consumed once
        const responseToCache = response.clone()
        caches.open(CACHE_NAME).then((cache) => {
          cache.put(event.request, responseToCache)
        })

        return response
      })
    }),
  )
})

// Special handling for map tile requests
async function handleTileRequest(request) {
  // First check if we have it in cache
  const cachedResponse = await caches.match(request)
  if (cachedResponse) {
    return cachedResponse
  }

  // If not in cache, try to get from network
  try {
    // Extract tile coordinates from the request URL
    // Format is /tiles/{z}/{x}/{y}.png
    const urlParts = request.url.split("/")
    const z = urlParts[urlParts.length - 3]
    const x = urlParts[urlParts.length - 2]
    const y = urlParts[urlParts.length - 1].split(".")[0]

    // Try to fetch from OpenStreetMap
    const osmUrl = `https://a.tile.openstreetmap.org/${z}/${x}/${y}.png`
    const networkResponse = await fetch(osmUrl)

    // Cache the response for future use
    const responseToCache = networkResponse.clone()
    const cache = await caches.open(TILE_CACHE_NAME)
    await cache.put(request, responseToCache)

    return networkResponse
  } catch (error) {
    // If network request fails, return the error tile
    return caches.match("/images/error-tile.png")
  }
}
