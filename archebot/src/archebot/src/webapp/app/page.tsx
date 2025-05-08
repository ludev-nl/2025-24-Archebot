"use client"

import { useState, useEffect } from "react"
import dynamic from "next/dynamic"
import { Button } from "@/components/ui/button"


// Import the map component dynamically to avoid SSR issues with Leaflet
const Map = dynamic(() => import("@/components/map"), {
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
  // State to hold the box coordinates
  const [Box, setBox] = useState<{
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

  const [path, setPath] = useState([]);


  // send box coordinates to the server when all corners are set and receive path planning coordinates
  useEffect(() => {
    const allCornersSet = Box.northEast && Box.southWest && Box.northWest && Box.southEast
  
    if (allCornersSet) {
      fetch("http://localhost:4000/box-coordinates", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(Box),
      })
        .then((res) => res.json())
        .then((data) => {
          console.log("Server response:", data)
          setPath(data.coordinates) // returned path planning coordinates
        })
        .catch((err) => {
          console.log(JSON.stringify(Box))
          console.error("Failed to send box coordinates:", err)
        })
    }
  }, [Box])

  // Send a start event to the server when the button is clicked
  const handleStart = () => {
    const allCornersSet = Box.northEast && Box.southWest && Box.northWest && Box.southEast
  
    if (!allCornersSet) {
      alert("Please draw a box on the map first.")
      return
    }
  
    fetch("http://localhost:4000/start", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({message: "start_clicked"}),
    })
      .then((res) => res.json())
      .then((data) => {
        console.log("Start response:", data)
      })
      .catch((err) => {
        console.error("Failed to send start event:", err)
      })
  }
  

  // State to manage service worker registration status
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
      <h1 className="text-3xl font-bold mb-6"> Geographic map</h1>
      <p className="mb-4 text-muted-foreground">
        Draw a rectangle on the map to get the latitude and longitude coordinates of the box and start the robot.
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

      {/* map with box */}
      <div className="space-y-6">
        <div className="card">
          <div className="card-header">
            <h2 className="card-title">Interactive Map</h2>
            <p className="card-description">
              Click the rectangle icon in the top left of the map, then draw a box by clicking and dragging.
            </p>
          </div>
          <div className="card-content">
            <div className="h-[600px] w-full rounded-md overflow-hidden border">
              <Map onBoxChange={setBox} path={path}/>
            </div>
          </div>
        </div>

        {/* Box Coordinates - now below the map */}
        <div className="card">
          <div className="card-header">
            <h2 className="card-title">Box Coordinates</h2>
            <p className="card-description">
              The coordinates of your selected area will appear here, 
              you can start the robot by clicking the start button
            </p>
          </div>
          <div className="card-content">
            {Box.southWest && Box.northEast && Box.southEast && Box.northWest ? (
              <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
                <div className="text-left mt-6">
                  <Button variant="outline" onClick={handleStart}>Start</Button>
                </div>
                <div>
                  <h3 className="font-medium mb-1">Southwest Corner:</h3>
                  <p className="text-sm">Latitude: {Box.southWest.lat.toFixed(6)}</p>
                  <p className="text-sm">Longitude: {Box.southWest.lng.toFixed(6)}</p>
                </div>
                <div>
                  <h3 className="font-medium mb-1">Northeast Corner:</h3>
                  <p className="text-sm">Latitude: {Box.northEast.lat.toFixed(6)}</p>
                  <p className="text-sm">Longitude: {Box.northEast.lng.toFixed(6)}</p>
                </div>
                <div>
                  <h3 className="font-medium mb-1">Southeast Corner:</h3>
                  <p className="text-sm">Latitude: {Box.southEast.lat.toFixed(6)}</p>
                  <p className="text-sm">Longitude: {Box.southEast.lng.toFixed(6)}</p>
                </div>
                <div>
                  <h3 className="font-medium mb-1">Northwest Corner:</h3>
                  <p className="text-sm">Latitude: {Box.northWest.lat.toFixed(6)}</p>
                  <p className="text-sm">Longitude: {Box.northWest.lng.toFixed(6)}</p>
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
    </main>
  )
}
