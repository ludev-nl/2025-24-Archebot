"use client"

import * as L from "leaflet";
import { useEffect, useState, useRef } from "react";
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";
import 'leaflet/dist/leaflet.css';

interface RectangleCorners {
  topLeft: [number, number]
  topRight: [number, number]
  bottomRight: [number, number]
  bottomLeft: [number, number]
}

const Map = () => {
  const [corners, setCorners] = useState<RectangleCorners | null>(null)
  const mapRef = useRef<L.Map | null>(null)
  const mapContainerRef = useRef<HTMLDivElement>(null)
  const [mapReady, setMapReady] = useState(false)

  // Insert the style once
  useEffect(() => {
    const styleElement = document.createElement("style")
    styleElement.innerHTML = `
      .corner-coordinates {
        position: absolute;
        bottom: 20px;
        left: 20px;
        z-index: 1000;
        background: white;
        padding: 10px;
        border-radius: 5px;
        box-shadow: 0 0 10px rgba(0,0,0,0.2);
        max-width: 300px;
      }
    `
    document.head.appendChild(styleElement)


    // cleanup in case component unmounts (good practice)
    return () => {
      document.head.removeChild(styleElement)
    }
  }, [])

  // Handle map initialization
  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) return

    // Create map instance manually instead of using MapContainer
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

    let currentRectangle: L.Rectangle | null = null

    const updateCorners = (layer: L.Rectangle) => {
      const bounds = layer.getBounds()
      const corners: RectangleCorners = {
        topLeft: [bounds.getNorthWest().lat, bounds.getNorthWest().lng],
        topRight: [bounds.getNorthEast().lat, bounds.getNorthEast().lng],
        bottomRight: [bounds.getSouthEast().lat, bounds.getSouthEast().lng],
        bottomLeft: [bounds.getSouthWest().lat, bounds.getSouthWest().lng],
      }
      setCorners(corners)
    }

    mapRef.current.on("pm:create", (e) => {
      if (e.shape === "Rectangle") {
        if (currentRectangle) {
          mapRef.current?.removeLayer(currentRectangle)
        }

        const layer = e.layer as L.Rectangle
        currentRectangle = layer
        updateCorners(layer)

        layer.pm.enable({ allowRotation: true })

        layer.on("pm:edit pm:rotate pm:drag", () => {
          updateCorners(layer)
        })
      }
    })

    setMapReady(true)

    // Clean up on unmount
    return () => {
      if (mapRef.current) {
        mapRef.current.remove()
        mapRef.current = null
      }
    }
  }, [])

  return (
    <div style={{ height: "100vh", width: "100%", position: "relative" }}>
      <div ref={mapContainerRef} style={{ height: "100%", width: "100%" }} />

      <div className="corner-coordinates">
        {corners ? (
          <>
            <h3>Rectangle Coordinates</h3>
            <p>Top Left: {corners.topLeft.join(", ")}</p>
            <p>Top Right: {corners.topRight.join(", ")}</p>
            <p>Bottom Right: {corners.bottomRight.join(", ")}</p>
            <p>Bottom Left: {corners.bottomLeft.join(", ")}</p>
          </>
        ) : (
          <p>Draw a rectangle on the map</p>
        )}
      </div>
    </div>
  )
}

export default Map;
