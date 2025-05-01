"use client"

import * as L from "leaflet"
import { useEffect, useState, useRef } from "react"
import "leaflet/dist/leaflet.css"
import "@geoman-io/leaflet-geoman-free"
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css"


interface MapProps {
  onBoxChange?: (box: {
    southWest: { lat: number; lng: number } | null
    northEast: { lat: number; lng: number } | null
    southEast: { lat: number; lng: number } | null
    northWest: { lat: number; lng: number } | null
  }) => void
}

const Map = ({ onBoxChange }: MapProps) => {
  const [box, setBox] = useState<{
    southWest: { lat: number; lng: number } | null
    northEast: { lat: number; lng: number } | null
    southEast: { lat: number; lng: number } | null
    northWest: { lat: number; lng: number } | null
  }>({
    southWest: null,
    northEast: null,
    southEast: null,
    northWest: null,
  })

  const mapRef = useRef<L.Map | null>(null)
  const mapContainerRef = useRef<HTMLDivElement>(null)
  const rectangleRef = useRef<L.Rectangle | null>(null)

  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) return

    // Initialize the map
    mapRef.current = L.map(mapContainerRef.current).setView([51.505, -0.09], 13)

    // Add tile layer
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    }).addTo(mapRef.current)

    // Initialize Geoman controls
    mapRef.current.pm.addControls({
      position: "topleft",
      drawMarker: false,
      drawCircleMarker: false,
      drawPolyline: false,
      drawRectangle: true,
      drawPolygon: false,
      drawCircle: false,
      editMode: true,
      dragMode: true,
      rotateMode: true,
      cutPolygon: false,
      removalMode: true,
    })

    // Handle rectangle creation and updates
    mapRef.current.on("pm:create", (e) => {
      if (e.shape === "Rectangle") {
        // Remove previous rectangle if exists
        if (rectangleRef.current) {
          mapRef.current?.removeLayer(rectangleRef.current)
        }

        const layer = e.layer as L.Rectangle
        rectangleRef.current = layer
        updateBoxCoordinates(layer)

        // Update coordinates on any modification
        layer.on("pm:edit pm:rotate pm:drag", () => {
          updateBoxCoordinates(layer)
        })

        // Handle rectangle removal
        layer.on("pm:remove", () => {
          rectangleRef.current = null
          setBox({
            southWest: null,
            northEast: null,
            southEast: null,
            northWest: null,
          })
          if (onBoxChange) onBoxChange({
            southWest: null,
            northEast: null,
            southEast: null,
            northWest: null,
          })
        })
      }
    })

    return () => {
      if (mapRef.current) {
        mapRef.current.remove()
        mapRef.current = null
      }
    }
  }, [onBoxChange])

  const updateBoxCoordinates = (layer: L.Rectangle) => {
    const bounds = layer.getBounds()
    const newBox = {
      southWest: { lat: bounds.getSouthWest().lat, lng: bounds.getSouthWest().lng },
      northEast: { lat: bounds.getNorthEast().lat, lng: bounds.getNorthEast().lng },
      southEast: { lat: bounds.getSouthEast().lat, lng: bounds.getSouthEast().lng },
      northWest: { lat: bounds.getNorthWest().lat, lng: bounds.getNorthWest().lng },
    }
    setBox(newBox)
    if (onBoxChange) onBoxChange(newBox)
  }

  return (
    <div style={{ height: "100vh", width: "100%", position: "relative" }}>
      <div ref={mapContainerRef} style={{ height: "100%", width: "100%" }} />

      <div className="corner-coordinates">
        <p>Draw a rectangle on the map</p>
      </div>
    </div>
  )
}

export default Map