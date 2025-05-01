"use client"

import { useEffect, useRef, useState } from "react"
import L from "leaflet"
import "leaflet/dist/leaflet.css"
import { useMap } from "react-leaflet"
import "react-leaflet-draw"

interface MapWithBoundingBoxProps {
  onBoundingBoxChange: (boundingBox: {
    southWest: { lat: number; lng: number } | null
    northEast: { lat: number; lng: number } | null
  }) => void
}

// This component ensures the map is properly sized after initialization
function MapInitializer() {
  const map = useMap()

  useEffect(() => {
    // Force a resize detection after the map is mounted
    setTimeout(() => {
      map.invalidateSize()
    }, 100)

    return () => {
      // Clean up any event listeners if needed
    }
  }, [map])

  return null
}

export default function MapWithBoundingBox({ onBoundingBoxChange }: MapWithBoundingBoxProps) {
  const mapRef = useRef<HTMLDivElement>(null)
  const leafletMapRef = useRef<L.Map | null>(null)
  const featureGroupRef = useRef<L.FeatureGroup | null>(null)
  const [isOnline, setIsOnline] = useState(true)
  const [mapInitialized, setMapInitialized] = useState(false)
  const [initError, setInitError] = useState<string | null>(null)

  // Set up online/offline detection
  useEffect(() => {
    setIsOnline(navigator.onLine)

    const handleOnline = () => setIsOnline(true)
    const handleOffline = () => setIsOnline(false)

    window.addEventListener("online", handleOnline)
    window.addEventListener("offline", handleOffline)

    return () => {
      window.removeEventListener("online", handleOnline)
      window.removeEventListener("offline", handleOffline)
    }
  }, [])

  // Set up Leaflet marker icons
  useEffect(() => {
    // Use local marker icons instead of CDN
  
    L.Icon.Default.mergeOptions({
      iconRetinaUrl: "/images/marker-icon-2x.png",
      iconUrl: "/images/marker-icon.png",
      shadowUrl: "/images/marker-shadow.png",
    })
  }, [])

  // Initialize the map
  useEffect(() => {
    if (!mapRef.current || leafletMapRef.current) return

    try {
      // Create the map instance
      const map = L.map(mapRef.current, {
        center: [40.7128, -74.006], // New York City coordinates
        zoom: 13,
      })

      leafletMapRef.current = map

      // Add the tile layer
      L.tileLayer(isOnline ? "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" : "/tiles/{z}/{x}/{y}.png", {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        errorTileUrl: "/images/error-tile.png",
      }).addTo(map)

      // Create feature group for drawing
      const featureGroup = new L.FeatureGroup()
      featureGroup.addTo(map)
      featureGroupRef.current = featureGroup

      // Initialize the draw control
      // Initialize the draw control
      const drawControl = new (L.Control as any).Draw({
        position: "topright",
        draw: {
          polyline: false,
          polygon: false,
          circle: false,
          circlemarker: false,
          marker: false,
          rectangle: true,
        },
        edit: {
          featureGroup: featureGroup,
        },
      })

      map.addControl(drawControl)

      // Set up event handlers
      map.on("draw:created", (e: any) => {
        const layer = e.layer
        featureGroup.addLayer(layer)

        if (layer instanceof L.Rectangle) {
          const bounds = layer.getBounds()
          const southWest = bounds.getSouthWest()
          const northEast = bounds.getNorthEast()

          onBoundingBoxChange({
            southWest: { lat: southWest.lat, lng: southWest.lng },
            northEast: { lat: northEast.lat, lng: northEast.lng },
          })
        }
      })

      map.on("draw:deleted", () => {
        onBoundingBoxChange({
          southWest: null,
          northEast: null,
        })
      })

      map.on("draw:edited", (e: any) => {
        const layers = e.layers
        layers.eachLayer((layer: any) => {
          if (layer instanceof L.Rectangle) {
            const bounds = layer.getBounds()
            const southWest = bounds.getSouthWest()
            const northEast = bounds.getNorthEast()

            onBoundingBoxChange({
              southWest: { lat: southWest.lat, lng: southWest.lng },
              northEast: { lat: northEast.lat, lng: northEast.lng },
            })
          }
        })
      })

      // Force a resize after initialization
      setTimeout(() => {
        map.invalidateSize()
        setMapInitialized(true)
      }, 200)
    } catch (error) {
      console.error("Error initializing map:", error)
      setInitError(error instanceof Error ? error.message : "Failed to initialize map")
    }

    // Cleanup function
    return () => {
      if (leafletMapRef.current) {
        leafletMapRef.current.remove()
        leafletMapRef.current = null
        featureGroupRef.current = null
      }
    }
  }, [onBoundingBoxChange, isOnline])

  // Handle online/offline status changes
  useEffect(() => {
    if (!leafletMapRef.current) return

    // Update tile layer when online status changes
    const map = leafletMapRef.current

    // Remove existing tile layers
    map.eachLayer((layer) => {
      if (layer instanceof L.TileLayer) {
        map.removeLayer(layer)
      }
    })

    // Add new tile layer based on online status
    L.tileLayer(isOnline ? "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" : "/tiles/{z}/{x}/{y}.png", {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      errorTileUrl: "/images/error-tile.png",
    }).addTo(map)
  }, [isOnline])

  // Function to manually reinitialize the map
  const reinitializeMap = () => {
    if (leafletMapRef.current) {
      leafletMapRef.current.remove()
      leafletMapRef.current = null
      featureGroupRef.current = null
    }

    setInitError(null)
    setMapInitialized(false)

    // Force a re-render to trigger the initialization effect
    setTimeout(() => {
      if (mapRef.current) {
        // Clear any existing content
        mapRef.current.innerHTML = ""

        // Re-trigger the initialization effect
        setIsOnline(navigator.onLine)
      }
    }, 100)
  }

  return (
    <div className="map-wrapper" style={{ position: "relative", height: "100%", width: "100%" }}>
      {/* Map container */}
      <div
        ref={mapRef}
        className="map-container"
        style={{ height: "100%", width: "100%" }}
        data-testid="map-container"
      />

      {/* Error message */}
      {initError && (
        <div
          className="map-error"
          style={{
            position: "absolute",
            top: "50%",
            left: "50%",
            transform: "translate(-50%, -50%)",
            backgroundColor: "rgba(255, 255, 255, 0.9)",
            padding: "1rem",
            borderRadius: "0.5rem",
            boxShadow: "0 2px 10px rgba(0, 0, 0, 0.1)",
            zIndex: 1000,
            textAlign: "center",
          }}
        >
          <p>Error: {initError}</p>
          <button
            onClick={reinitializeMap}
            className="button button-primary"
            style={{ marginTop: "0.5rem", fontSize: "0.875rem" }}
          >
            Retry
          </button>
        </div>
      )}

      {/* Loading indicator */}
      {!mapInitialized && !initError && (
        <div
          style={{
            position: "absolute",
            top: "50%",
            left: "50%",
            transform: "translate(-50%, -50%)",
            zIndex: 1000,
          }}
        >
          <p>Loading map...</p>
        </div>
      )}

      {/* Reload button */}
      <button
        onClick={reinitializeMap}
        className="map-reload-button"
        style={{
          position: "absolute",
          bottom: "10px",
          right: "10px",
          zIndex: 1000,
          backgroundColor: "white",
          border: "1px solid #ccc",
          borderRadius: "4px",
          padding: "4px 8px",
          fontSize: "12px",
          cursor: "pointer",
        }}
      >
        Reload Map
      </button>
    </div>
  )
}
