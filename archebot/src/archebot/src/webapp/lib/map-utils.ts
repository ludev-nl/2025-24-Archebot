// Convert latitude and longitude to tile coordinates
export const latLngToTile = (lat: number, lng: number, zoom: number) => {
  const n = Math.pow(2, zoom)
  const x = Math.floor(((lng + 180) / 360) * n)
  const y = Math.floor(
    ((1 - Math.log(Math.tan((lat * Math.PI) / 180) + 1 / Math.cos((lat * Math.PI) / 180)) / Math.PI) / 2) * n,
  )
  return { x, y }
}

// Calculate the number of tiles needed for the given bounds and zoom levels
export const calculateTotalTiles = (
  bounds: {
    southWest: { lat: number; lng: number }
    northEast: { lat: number; lng: number }
  },
  minZoom: number,
  maxZoom: number,
) => {
  let total = 0

  for (let zoom = minZoom; zoom <= maxZoom; zoom++) {
    // Convert lat/lng to tile coordinates
    const nwTile = latLngToTile(bounds.northEast.lat, bounds.southWest.lng, zoom)
    const seTile = latLngToTile(bounds.southWest.lat, bounds.northEast.lng, zoom)

    // Calculate number of tiles in this zoom level
    const tilesX = Math.abs(seTile.x - nwTile.x) + 1
    const tilesY = Math.abs(seTile.y - nwTile.y) + 1
    total += tilesX * tilesY
  }

  return total
}
