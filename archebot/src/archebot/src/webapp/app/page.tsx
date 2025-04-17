"use client"

import { useState, useEffect } from "react"
import dynamic from "next/dynamic"

// Import the map component dynamically to avoid SSR issues with Leaflet
const MapWithBoundingBox = dynamic(() => import("@/components/map-with-bounding-box"), {
  ssr: false,
  loading: () => (
    <div className="h-[500px] w-full flex items-center justify-center bg-muted">
      <p>Loading map...</p>
    </div>
  ),
})

// Import the offline tile manager component
const OfflineTileManager = dynamic(() => import("@/components/offline-tile-manager"), {
  ssr: false,
})

// Import the offline map notice component
const OfflineMapNotice = dynamic(() => import("@/components/offline-map-notice"), {
  ssr: false,
})

export default function Home() {
  const [boundingBox, setBoundingBox] = useState<{
    southWest: { lat: number; lng: number } | null
    northEast: { lat: number; lng: number } | null
  }>({
    southWest: null,
    northEast: null,
  })

  const [serviceWorkerStatus, setServiceWorkerStatus] = useState<"loading" | "registered" | "failed">("loading")

  // Register service worker for offline functionality
  useEffect(() => {
    // Only attempt to register service worker in production or when running locally
    // Skip registration in preview environments
    const isPreviewEnvironment = window.location.hostname.includes("vusercontent.net")

    if ("serviceWorker" in navigator && !isPreviewEnvironment) {
      setServiceWorkerStatus("loading")

      // Use a try-catch block to handle potential errors
      try {
        navigator.serviceWorker
          .register("/service-worker.js")
          .then((registration) => {
            console.log("Service Worker registered with scope:", registration.scope)
            setServiceWorkerStatus("registered")
          })
          .catch((error) => {
            console.error("Service Worker registration failed:", error)
            setServiceWorkerStatus("failed")
          })
      } catch (error) {
        console.error("Error during service worker registration:", error)
        setServiceWorkerStatus("failed")
      }
    } else {
      // If service workers aren't supported or we're in a preview environment
      console.log("Service Workers not supported or skipped in preview environment")
      setServiceWorkerStatus("failed")
    }
  }, [])

  return (
    <main className="container mx-auto py-8 px-4">
      <h1 className="text-3xl font-bold mb-6">Offline Geographic Bounding Box Selector</h1>
      <p className="mb-4 text-muted-foreground">
        Draw a rectangle on the map to get the latitude and longitude coordinates of the bounding box.
      </p>

      {serviceWorkerStatus === "failed" && (
        <div className="alert mb-4 p-4 bg-muted border rounded-md">
          <p className="text-sm">
            <strong>Note:</strong> Offline mode is limited in this preview environment. For full offline functionality,
            deploy the application to a production environment.
          </p>
        </div>
      )}

      <OfflineMapNotice />

      <div className="grid gap-6 md:grid-cols-3">
        <div className="md:col-span-2">
          <div className="card">
            <div className="card-header">
              <h2 className="card-title">Interactive Map</h2>
              <p className="card-description">
                Click the rectangle icon in the top right of the map, then draw a box by clicking and dragging.
              </p>
            </div>
            <div className="card-content">
              <div className="h-[500px] w-full rounded-md overflow-hidden border">
                <MapWithBoundingBox onBoundingBoxChange={setBoundingBox} />
              </div>
            </div>
          </div>
        </div>

        <div className="space-y-6">
          <div className="card">
            <div className="card-header">
              <h2 className="card-title">Bounding Box Coordinates</h2>
              <p className="card-description">The coordinates of your selected area will appear here</p>
            </div>
            <div className="card-content">
              {boundingBox.southWest && boundingBox.northEast ? (
                <div className="space-y-4">
                  <div>
                    <h3 className="font-medium mb-1">Southwest Corner:</h3>
                    <p className="text-sm">Latitude: {boundingBox.southWest.lat.toFixed(6)}</p>
                    <p className="text-sm">Longitude: {boundingBox.southWest.lng.toFixed(6)}</p>
                  </div>
                  <div>
                    <h3 className="font-medium mb-1">Northeast Corner:</h3>
                    <p className="text-sm">Latitude: {boundingBox.northEast.lat.toFixed(6)}</p>
                    <p className="text-sm">Longitude: {boundingBox.northEast.lng.toFixed(6)}</p>
                  </div>
                  <div className="pt-2 border-t">
                    <h3 className="font-medium mb-1">For PostGIS Query:</h3>
                    <div className="bg-muted p-2 rounded-md text-xs overflow-x-auto">
                      <pre>
                        {`ST_MakeBox2D(
ST_Point(${boundingBox.southWest.lng.toFixed(6)}, ${boundingBox.southWest.lat.toFixed(6)}), 
ST_Point(${boundingBox.northEast.lng.toFixed(6)}, ${boundingBox.northEast.lat.toFixed(6)})
)`}
                      </pre>
                    </div>
                  </div>
                </div>
              ) : (
                <div className="text-center py-8 text-muted-foreground">
                  <p>Draw a box on the map to see coordinates</p>
                </div>
              )}
            </div>
          </div>

          {/* Only show the tile manager if service worker is registered successfully */}
          {serviceWorkerStatus !== "failed" && <OfflineTileManager />}

          {serviceWorkerStatus === "failed" && (
            <div className="card">
              <div className="card-header">
                <h2 className="card-title">Offline Functionality</h2>
              </div>
              <div className="card-content">
                <p className="text-sm text-muted-foreground">
                  Full offline functionality is not available in this preview environment. The application will still
                  work, but requires an internet connection for map tiles.
                </p>
              </div>
            </div>
          )}
        </div>
      </div>
    </main>
  )
}
