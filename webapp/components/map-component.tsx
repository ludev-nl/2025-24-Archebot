"use client"

import { useState } from "react"
import { MapContainer, TileLayer, Marker, Popup, useMap } from "react-leaflet"
import "leaflet/dist/leaflet.css"
import { Icon } from "leaflet"
import { Button } from "@/components/ui/button"

// Fix for default marker icon in Leaflet with Next.js
const customIcon = new Icon({
  iconUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png",
  iconRetinaUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon-2x.png",
  shadowUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-shadow.png",
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  shadowSize: [41, 41],
})

// Sample locations
const locations = [
  { id: 1, name: "New York", position: [40.7128, -74.006], description: "The Big Apple" },
  { id: 2, name: "London", position: [51.5074, -0.1278], description: "The capital of England" },
  { id: 3, name: "Tokyo", position: [35.6762, 139.6503], description: "Japan's busy capital" },
  { id: 4, name: "Sydney", position: [-33.8688, 151.2093], description: "Australia's famous harbor city" },
  { id: 5, name: "Rio de Janeiro", position: [-22.9068, -43.1729], description: "Brazil's seaside city" },
]

// Component to recenter the map
function SetViewOnClick({ coords }: { coords: [number, number] }) {
  const map = useMap()
  map.setView(coords, map.getZoom())
  return null
}

export default function MapComponent() {
  const [center, setCenter] = useState<[number, number]>([20, 0])
  const [selectedLocation, setSelectedLocation] = useState<number | null>(null)

  // Handle location selection
  const handleLocationSelect = (id: number) => {
    const location = locations.find((loc) => loc.id === id)
    if (location) {
      setCenter(location.position as [number, number])
      setSelectedLocation(id)
    }
  }

  return (
    <div className="h-full w-full">
      <div className="bg-background p-2 mb-2 rounded-md flex gap-2 overflow-x-auto">
        {locations.map((location) => (
          <Button
            key={location.id}
            variant={selectedLocation === location.id ? "default" : "outline"}
            size="sm"
            onClick={() => handleLocationSelect(location.id)}
          >
            {location.name}
          </Button>
        ))}
      </div>

      <MapContainer center={center} zoom={3} style={{ height: "calc(100% - 50px)", width: "100%" }} className="z-0">
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />

        {locations.map((location) => (
          <Marker key={location.id} position={location.position as [number, number]} icon={customIcon}>
            <Popup>
              <div>
                <h3 className="font-bold">{location.name}</h3>
                <p>{location.description}</p>
              </div>
            </Popup>
          </Marker>
        ))}

        <SetViewOnClick coords={center} />
      </MapContainer>
    </div>
  )
}

