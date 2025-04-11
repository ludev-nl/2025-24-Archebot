"use client"

import { useEffect, useRef, useState } from "react"
import L from "leaflet"
import "leaflet/dist/leaflet.css"
import { MapContainer, TileLayer, FeatureGroup } from "react-leaflet"
import { EditControl } from "react-leaflet-draw"

interface MapWithBoundingBoxProps {
  onBoundingBoxChange: (boundingBox: {
    southWest: { lat: number; lng: number } | null
    northEast: { lat: number; lng: number } | null
  }) => void
}

export default function MapWithBoundingBox({ onBoundingBoxChange }: MapWithBoundingBoxProps) {
  const featureGroupRef = useRef<L.FeatureGroup>(null)
  const [isOnline, setIsOnline] = useState(true)

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

    return () => {
      window.removeEventListener("online", handleOnline)
      window.removeEventListener("offline", handleOffline)
    }
  }, [])

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

  const handleDeleted = () => {
    onBoundingBoxChange({
      southWest: null,
      northEast: null,
    })
  }

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

  return (
    <MapContainer
      center={[40.7128, -74.006]} // New York City coordinates
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
