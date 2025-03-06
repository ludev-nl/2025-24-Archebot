import MapComponent from "@/components/map-component"

export default function Home() {
  return (
    <main className="flex min-h-screen flex-col items-center justify-between p-4">
      <div className="w-full max-w-5xl">
        <h1 className="text-3xl font-bold mb-4">Interactive Map</h1>
        <p className="mb-6">Explore the map below. Click on markers to see more information.</p>
        <div className="h-[600px] w-full rounded-lg overflow-hidden border border-gray-200">
          <MapComponent />
        </div>
      </div>
    </main>
  )
}

