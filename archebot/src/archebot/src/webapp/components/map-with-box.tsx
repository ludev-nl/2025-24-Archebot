"use client"

import { useEffect, useRef, useState } from "react"
import L from "leaflet"
import "leaflet/dist/leaflet.css"
import "leaflet-draw/dist/leaflet.draw.css"
import "leaflet-draw"

export interface BoxCoordinates {
  southWest: { lat: number; lng: number } | null
  northEast: { lat: number; lng: number } | null
  southEast: { lat: number; lng: number } | null
  northWest: { lat: number; lng: number } | null
}

interface MapWithBoxProps {
  onBoxChange: (Box: BoxCoordinates) => void
}

export default function MapWithBox({ onBoxChange }: MapWithBoxProps) {
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

  // Add CSS to fix drawing controls
  useEffect(() => {
    // Create a style element
    const style = document.createElement("style")

    // Add CSS rules to fix drawing controls and position them more to the left
    style.textContent = `
      /* Position the draw control on the left side */
      .leaflet-draw.leaflet-control {
        position: absolute;
        left: 10px;
        top: 80px;
        z-index: 1000;
      }
      
      /* Fix drawing control text overlap */
      .leaflet-draw-toolbar {
        margin-top: 0 !important;
      }
      
      /* Ensure the draw actions menu doesn't get cut off */
      .leaflet-draw-actions {
        left: 35px !important;
        top: 0 !important;
      }
      
      /* Style the action buttons */
      .leaflet-draw-actions a {
        display: block !important;
        padding: 4px 10px !important;
        background-color: white !important;
        color: #333 !important;
        font-size: 12px !important;
        line-height: 20px !important;
        text-decoration: none !important;
        border-bottom: 1px solid #ccc !important;
      }
      
      /* Fix tooltip positioning */
      .leaflet-draw-tooltip {
        background: white !important;
        border: 1px solid #999 !important;
        border-radius: 4px !important;
        color: #333 !important;
        font-size: 12px !important;
        padding: 4px 8px !important;
        white-space: nowrap !important;
        z-index: 1000 !important;
      }
      
      /* Ensure the edit actions don't get cut off */
      .leaflet-draw-edit-edit .leaflet-draw-actions,
      .leaflet-draw-edit-remove .leaflet-draw-actions {
        left: 35px !important;
        top: 0 !important;
      }
      
      /* Fix for the edit toolbar */
      .leaflet-draw-toolbar-top {
        margin-top: 0 !important;
      }
    `

    // Add the style element to the document head
    document.head.appendChild(style)

    // Clean up function to remove the style element when component unmounts
    return () => {
      document.head.removeChild(style)
    }
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
      const drawControl = new (L.Control as any).Draw({
        position: "topleft", // Changed from topright to topleft
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
          const southEast = bounds.getSouthEast()
          const northWest = bounds.getNorthWest()

          onBoxChange({
            southWest: { lat: southWest.lat, lng: southWest.lng },
            northEast: { lat: northEast.lat, lng: northEast.lng },
            southEast: { lat: southEast.lat, lng: southEast.lng },
            northWest: { lat: northWest.lat, lng: northWest.lng },
          })
        }
      })

      map.on("draw:deleted", () => {
        onBoxChange({
          southWest: null,
          northEast: null,
          southEast: null,
          northWest: null,
        })
      })

      map.on("draw:edited", (e: any) => {
        const layers = e.layers
        layers.eachLayer((layer: any) => {
          if (layer instanceof L.Rectangle) {
            const bounds = layer.getBounds()
            const southWest = bounds.getSouthWest()
            const northEast = bounds.getNorthEast()
            const southEast = bounds.getSouthEast()
            const northWest = bounds.getNorthWest()

            onBoxChange({
              southWest: { lat: southWest.lat, lng: southWest.lng },
              northEast: { lat: northEast.lat, lng: northEast.lng },
              southEast: { lat: southEast.lat, lng: southEast.lng },
              northWest: { lat: northWest.lat, lng: northWest.lng },
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
  }, [onBoxChange, isOnline])

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
