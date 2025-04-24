import { useEffect, useRef, useState } from "react"
import L from "leaflet"
import "leaflet/dist/leaflet.css"
import "leaflet-draw/dist/leaflet.draw.css"
import "leaflet-draw"
import "leaflet-path-transform"
import "leaflet-draw-rotate"

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
  const currentRectRef = useRef<L.Rectangle | null>(null)
  const rotateHandleRef = useRef<any>(null)

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

  // Function to update coordinates from rectangle
  const updateCoordinates = (layer: L.Rectangle) => {
    const latlngs = layer.getLatLngs()[0] as L.LatLng[]
    onBoxChange({
      northWest: { lat: latlngs[0].lat, lng: latlngs[0].lng },
      northEast: { lat: latlngs[1].lat, lng: latlngs[1].lng },
      southEast: { lat: latlngs[2].lat, lng: latlngs[2].lng },
      southWest: { lat: latlngs[3].lat, lng: latlngs[3].lng },
    })
  }

  // Initialize the map
  useEffect(() => {
    if (!mapRef.current || leafletMapRef.current) return

    try {
      const map = L.map(mapRef.current, {
        center: [40.7128, -74.006], // New York City coordinates
        zoom: 13,
      })

      leafletMapRef.current = map

      L.tileLayer(isOnline ? "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" : "/tiles/{z}/{x}/{y}.png", {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        errorTileUrl: "/images/error-tile.png",
      }).addTo(map)

      const featureGroup = new L.FeatureGroup()
      featureGroup.addTo(map)
      featureGroupRef.current = featureGroup

      const drawControl = new (L.Control as any).Draw({
        position: "topleft",
        draw: {
          polyline: false,
          polygon: false,
          circle: false,
          circlemarker: false,
          marker: false,
          rectangle: {
            showArea: true,
            shapeOptions: {
              transform: true // Enable transformation
            }
          },
        },
        edit: {
          featureGroup: featureGroup,
          edit: {
            selectedPathOptions: {
              transform: {
                rotation: true, // Enable rotation
                scaling: false  // Disable scaling if not needed
              }
            }
          }
        },
      })

      map.addControl(drawControl)

      // Handle rectangle creation
      map.on("draw:created", (e: any) => {
        const layer = e.layer
        if (currentRectRef.current) {
          console.log("Removing previous rectangle")
          featureGroup.removeLayer(currentRectRef.current)
          if (rotateHandleRef.current) {
            map.removeLayer(rotateHandleRef.current)
          }
        }

        if (layer instanceof L.Rectangle) {
          featureGroup.addLayer(layer)
          currentRectRef.current = layer;

          // Enable transformation with rotation
          (layer as any).transform.enable({
            rotation: true,
            scaling: false,
          })

          // Add rotate handle
          rotateHandleRef.current = new (L as any).pathTransform.RotateHandle({
            path: layer,
            icon: new L.DivIcon({
              className: 'leaflet-rotate-handle',
              iconSize: [16, 16]
            }),
            position: 'topright'
          }).addTo(map)

          // Update coordinates initially
          updateCoordinates(layer)

          // Listen for transformation events
          layer.on('transform', () => {
            updateCoordinates(layer)
          })
        }
      })

      // Handle rectangle deletion
      map.on("draw:deleted", () => {
        currentRectRef.current = null
        if (rotateHandleRef.current) {
          map.removeLayer(rotateHandleRef.current)
          rotateHandleRef.current = null
        }
        onBoxChange({
          southWest: null,
          northEast: null,
          southEast: null,
          northWest: null,
        })
      })

      // Handle editing (including rotation)
      map.on("draw:edited", (e: any) => {
        e.layers.eachLayer((layer: any) => {
          if (layer instanceof L.Rectangle) {
            updateCoordinates(layer)
          }
        })
      })

      setTimeout(() => {
        map.invalidateSize()
        setMapInitialized(true)
      }, 200)
    } catch (error) {
      console.error("Error initializing map:", error)
      setInitError(error instanceof Error ? error.message : "Failed to initialize map")
    }

    return () => {
      if (leafletMapRef.current) {
        leafletMapRef.current.remove()
        leafletMapRef.current = null
        featureGroupRef.current = null
      }
    }
  }, [onBoxChange, isOnline])

  const reinitializeMap = () => {
    if (leafletMapRef.current) {
      leafletMapRef.current.remove()
      leafletMapRef.current = null
      featureGroupRef.current = null
    }

    setInitError(null)
    setMapInitialized(false)

    setTimeout(() => {
      if (mapRef.current) {
        mapRef.current.innerHTML = ""
        setIsOnline(navigator.onLine)
      }
    }, 100)
  }

  return (
    <div className="map-wrapper" style={{ position: "relative", height: "100%", width: "100%" }}>
      <style>
      {`
        .leaflet-rotate-handle {
          background-color: #fff;
          border: 2px solid #3388ff;
          border-radius: 50%;
          width: 16px;
          height: 16px;
          cursor: url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24"><path fill="%233388ff" d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.42 0-8-3.58-8-8s3.58-8 8-8 8 3.58 8 8-3.58 8-8 8z"/><path fill="%233388ff" d="M12 8l-4 4h3v4h2v-4h3l-4-4z"/></svg>') 8 8, pointer;
        }
      `}
      </style>
      <div
        ref={mapRef}
        className="map-container"
        style={{ height: "100%", width: "100%" }}
        data-testid="map-container"
      />
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