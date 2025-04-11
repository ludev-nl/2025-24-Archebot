"use client"

import { useEffect, useRef, useState } from "react"
import L from "leaflet"
import "leaflet/dist/leaflet.css"
import { MapContainer, TileLayer, FeatureGroup } from "react-leaflet"
import { EditControl } from "react-leaflet-draw"

// define the box coordinates called when coords are created, edited or deleted
interface MapWithBoundingBoxProps {
  onBoundingBoxChange: (boundingBox: {
    southWest: { lat: number; lng: number } | null
    northEast: { lat: number; lng: number } | null
  }) => void
}

// This component renders a Leaflet map with a drawable box feature
export default function MapWithBoundingBox({ onBoundingBoxChange }: MapWithBoundingBoxProps) {
  const featureGroupRef = useRef<L.FeatureGroup>(null) // reference to drawn features on the map
  const [isOnline, setIsOnline] = useState(true) // keep track of online status

  // runs only once when the component mounts
  useEffect(() => {
    // Set initial online status
    setIsOnline(navigator.onLine)

    // Add event listeners for online/offline status
    const handleOnline = () => setIsOnline(true)
    const handleOffline = () => setIsOnline(false)

    window.addEventListener("online", handleOnline)
    window.addEventListener("offline", handleOffline)

    // Use local marker icons instead of CDN
    L.Icon.Default.mergeOptions({
      iconRetinaUrl: "/images/marker-icon-2x.png",
      iconUrl: "/images/marker-icon.png",
      shadowUrl: "/images/marker-shadow.png",
    })

    // Cleanup event listeners on component unmount
    return () => {
      window.removeEventListener("online", handleOnline)
      window.removeEventListener("offline", handleOffline)
    }
  }, [])

  // Extract box coordiantes and call onboundingBoxChange to save coords to props
  const handleCreated = (e: any) => {
    const layer = e.layer
    if (layer instanceof L.Rectangle) {
      const bounds = layer.getBounds()
      const southWest = bounds.getSouthWest()
      const northEast = bounds.getNorthEast()

      onBoundingBoxChange({
        southWest: { lat: southWest.lat, lng: southWest.lng },
        northEast: { lat: northEast.lat, lng: northEast.lng },
      })
    }
  }

  // Clear the bounding box when deleted
  const handleDeleted = () => {
    onBoundingBoxChange({
      southWest: null,
      northEast: null,
    })
  }

  // Update the bounding box when edited
  const handleEdited = (e: any) => {
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
  }

  // Render the map with a drawable bounding box feature
  return (
    <MapContainer
      center={[40.7128, -74.006]} // New York City coordinates default
      zoom={13}
      style={{ height: "100%", width: "100%" }}
    >
      {/* Use online tiles when online, fallback to local tiles when available */}
      <TileLayer
        url={isOnline ? "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" : "/tiles/{z}/{x}/{y}.png"}
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        errorTileUrl="/images/error-tile.png"
      />
      <FeatureGroup ref={featureGroupRef}>
        <EditControl
          position="topright"
          onCreated={handleCreated}
          onDeleted={handleDeleted}
          onEdited={handleEdited}
          draw={{
            polyline: false,
            polygon: false,
            circle: false,
            circlemarker: false,
            marker: false,
            rectangle: true,
          }}
        />
      </FeatureGroup>
    </MapContainer>
  )
}
