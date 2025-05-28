"use client"
require('dotenv').config();
const API_BASE_URL = process.env.NEXT_PUBLIC_API_BASE_URL;

import { useState, useEffect } from "react";
import dynamic from "next/dynamic";
import { Button } from "@/components/ui/button";

// Import the map component dynamically to avoid SSR issues with Leaflet
const Map = dynamic(() => import("@/components/map"), {
  ssr: false,
  loading: () => (
    <div className="h-[500px] w-full flex items-center justify-center bg-muted">
      <p>Loading map...</p>
    </div>
  ),
})


type Shard = {
  id: number;
  latitude: number;
  longitude: number;
  photo?: string; // optional if sometimes missing
};

export type ShardInfo = {
  lat: number;
  lng: number;
  image: string | null; // Use optional chaining to handle missing photo
};

export default function Home() {

  // State to hold the box coordinates
  const [Box, setBox] = useState<{
    southWest: L.LatLng | null;
    northEast: L.LatLng | null;
    southEast: L.LatLng | null;
    northWest: L.LatLng | null;
  }>({
    southWest: null,
    northEast: null,
    southEast: null,
    northWest: null,
  })

  const [path, setPath] = useState([]); // State to hold the path planning coordinates

  // send box coordinates to the server when all corners are set and receive path planning coordinates
  useEffect(() => {
    const allCornersSet = Box.northEast && Box.southWest && Box.northWest && Box.southEast
    console.log("Box coordinates:", Box)
    if (allCornersSet) {
      fetch(`${API_BASE_URL}/box-coordinates`, {
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

  const [shards, setShards] = useState<ShardInfo[]>([]); // State to hold the shards
  
  // Fetch shard locations from the server every 10 seconds
  useEffect(() => {
    const fetchShards = () => {
      fetch(`${API_BASE_URL}/shards`)
        .then((res) => res.json())
        .then((data) => {
          const simplifiedShards = data.map((shard: Shard) => ({
            lat: shard.latitude,
            lng: shard.longitude,
            image: shard.photo || null, // Use optional chaining to handle missing photo
          }));

          setShards(simplifiedShards)
          console.log("Fetched shards:", simplifiedShards)
        })
        .catch((err) => {
          console.error("Failed to fetch shards:", err)
        })
    }
    
    fetchShards() // initial fetch
    const intervalId = setInterval(fetchShards, 10000)
    return () => clearInterval(intervalId)
  }, [])


  // Send a start event to the server when the start button is clicked
  const handleStart = () => {
    const allCornersSet = Box.northEast && Box.southWest && Box.northWest && Box.southEast
  
    if (!allCornersSet) {
      alert("Please draw a box on the map first.")
      return
    }
  
    fetch(`${API_BASE_URL}/start`, {
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
  
  return (
    <main className="container mx-auto py-8 px-4">
      <h1 className="text-3xl font-bold mb-6"> Geographic map</h1>
      <p className="mb-4 text-muted-foreground">
        Draw a rectangle on the map to get the latitude and longitude coordinates of the box and start the robot.
      </p>

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
              <Map onBoxChange={setBox} path={path} shards={shards}/>
            </div>
          </div>
        </div>

        {/* Box Coordinates and start button below the map */}
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
      </div>
    </main>
  )
}
