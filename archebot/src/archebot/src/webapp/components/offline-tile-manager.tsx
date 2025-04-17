

"use client"

import { useState } from "react"
import { latLngToTile, calculateTotalTiles } from "@/utils/map-utils"

// Define the area to cache (New York City by default)
const DEFAULT_BOUNDS = {
  southWest: { lat: 40.7, lng: -74.05 },
  northEast: { lat: 40.73, lng: -73.97 },
}

// Define zoom levels to cache
const MIN_ZOOM = 12
const MAX_ZOOM = 15

export default function OfflineTileManager() {
  const [isCaching, setIsCaching] = useState(false)
  const [progress, setProgress] = useState(0)
  const [totalTiles, setTotalTiles] = useState(0)
  const [cachedTiles, setCachedTiles] = useState(0)
  const [isComplete, setIsComplete] = useState(false)
  const [error, setError] = useState<string | null>(null)

  // Cache tiles for offline use
  const cacheTiles = async () => {
    if (isCaching) return

    setError(null)
    setIsCaching(true)
    setProgress(0)
    setIsComplete(false)

    // Check if caches API is available
    if (!("caches" in window)) {
      setError("Cache API is not available in this browser or environment")
      setIsCaching(false)
      return
    }

    // Calculate total tiles
    const total = calculateTotalTiles(DEFAULT_BOUNDS, MIN_ZOOM, MAX_ZOOM)
    setTotalTiles(total)
    setCachedTiles(0)

    let cached = 0

    try {
      // Create a cache for map tiles
      const cache = await caches.open("map-tiles")

      // Loop through zoom levels
      for (let zoom = MIN_ZOOM; zoom <= MAX_ZOOM; zoom++) {
        // Convert lat/lng to tile coordinates
        const nwTile = latLngToTile(DEFAULT_BOUNDS.northEast.lat, DEFAULT_BOUNDS.southWest.lng, zoom)
        const seTile = latLngToTile(DEFAULT_BOUNDS.southWest.lat, DEFAULT_BOUNDS.northEast.lng, zoom)

        // Loop through all tiles in the current zoom level
        for (let x = Math.min(nwTile.x, seTile.x); x <= Math.max(nwTile.x, seTile.x); x++) {
          for (let y = Math.min(nwTile.y, seTile.y); y <= Math.max(nwTile.y, seTile.y); y++) {
            // Construct the tile URL (using OSM format)
            const tileUrl = `https://a.tile.openstreetmap.org/${zoom}/${x}/${y}.png`

            try {
              // Fetch the tile and store it in the cache
              await cache.add(tileUrl)

              // Update progress
              cached++
              setCachedTiles(cached)
              setProgress(Math.floor((cached / total) * 100))
            } catch (error) {
              console.error(`Failed to cache tile: ${tileUrl}`, error)
            }
          }
        }
      }

      setIsComplete(true)
    } catch (error) {
      console.error("Error caching tiles:", error)
      setError("Failed to cache tiles. This may be due to browser restrictions or network issues.")
    } finally {
      setIsCaching(false)
    }
  }

  return (
    <div className="card">
      <div className="card-header">
        <h2 className="card-title">Offline Map Tiles</h2>
      </div>
      <div className="card-content space-y-4">
        <div className="text-sm text-muted-foreground">
          Cache map tiles for offline use. This will download approximately{" "}
          {calculateTotalTiles(DEFAULT_BOUNDS, MIN_ZOOM, MAX_ZOOM)} tiles.
        </div>

        {error && <div className="p-3 text-sm bg-muted border rounded-md text-muted-foreground">{error}</div>}

        {isCaching && (
          <div className="space-y-2">
            <div className="progress">
              <div className="progress-indicator" style={{ transform: `translateX(-${100 - progress}%)` }}></div>
            </div>
            <div className="text-xs text-muted-foreground">
              Caching {cachedTiles} of {totalTiles} tiles ({progress}%)
            </div>
          </div>
        )}

        <button
          onClick={cacheTiles}
          disabled={isCaching}
          className={`button w-full ${isComplete ? "button-outline" : "button-primary"}`}
          data-testid="cache-tiles-button"
        >
          {isComplete ? (
            <span className="flex items-center justify-center">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="16"
                height="16"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
                className="mr-2"
              >
                <polyline points="20 6 9 17 4 12"></polyline>
              </svg>
              Tiles Cached
            </span>
          ) : (
            <span className="flex items-center justify-center">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="16"
                height="16"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
                className="mr-2"
              >
                <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path>
                <polyline points="7 10 12 15 17 10"></polyline>
                <line x1="12" y1="15" x2="12" y2="3"></line>
              </svg>
              Cache Map Tiles
            </span>
          )}
        </button>
      </div>
    </div>
  )
}
