import { render, screen, fireEvent, waitFor } from "@testing-library/react"
import OfflineTileManager from "@/components/offline-tile-manager"

// Mock the caches API
const mockCache = {
  add: jest.fn().mockResolvedValue(undefined),
}

const mockCaches = {
  open: jest.fn().mockResolvedValue(mockCache),
}

Object.defineProperty(window, "caches", {
  value: mockCaches,
  writable: true,
})

describe("OfflineTileManager", () => {
  beforeEach(() => {
    jest.clearAllMocks()
  })

  test("renders the tile manager", () => {
    render(<OfflineTileManager />)
    expect(screen.getByText("Offline Map Tiles")).toBeInTheDocument()
    expect(screen.getByText(/Cache map tiles for offline use/)).toBeInTheDocument()
    expect(screen.getByRole("button", { name: /Cache Map Tiles/i })).toBeInTheDocument()
  })

  test("shows error when caches API is not available", async () => {
    // Remove caches API
    Object.defineProperty(window, "caches", {
      value: undefined,
      writable: true,
    })

    render(<OfflineTileManager />)

    // Click the cache button
    fireEvent.click(screen.getByRole("button", { name: /Cache Map Tiles/i }))

    // Should show error message
    await waitFor(() => {
      expect(screen.getByText("Cache API is not available in this browser or environment")).toBeInTheDocument()
    })

    // Restore caches API for other tests
    Object.defineProperty(window, "caches", {
      value: mockCaches,
      writable: true,
    })
  })

  test("shows progress when caching tiles", async () => {
    render(<OfflineTileManager />)

    // Click the cache button
    fireEvent.click(screen.getByRole("button", { name: /Cache Map Tiles/i }))

    // Should show progress
    await waitFor(() => {
      expect(screen.getByText(/Caching 0 of/)).toBeInTheDocument()
    })
  })

  test("shows completion message after caching", async () => {
    render(<OfflineTileManager />)

    // Click the cache button
    fireEvent.click(screen.getByRole("button", { name: /Cache Map Tiles/i }))

    // Wait for caching to complete
    await waitFor(() => {
      expect(mockCaches.open).toHaveBeenCalledWith("map-tiles")
      expect(mockCache.add).toHaveBeenCalled()
    })

    // Should show completion message
    await waitFor(() => {
      expect(screen.getByRole("button", { name: /Tiles Cached/i })).toBeInTheDocument()
    })
  })

  test("handles cache error", async () => {
    // Make cache.add reject
    mockCache.add.mockRejectedValueOnce(new Error("Cache error"))

    render(<OfflineTileManager />)

    // Click the cache button
    fireEvent.click(screen.getByRole("button", { name: /Cache Map Tiles/i }))

    // Wait for caching to attempt
    await waitFor(() => {
      expect(mockCaches.open).toHaveBeenCalledWith("map-tiles")
      expect(mockCache.add).toHaveBeenCalled()
    })

    // Should continue without error (individual tile errors are caught)
    await waitFor(() => {
      expect(screen.getByRole("button", { name: /Tiles Cached/i })).toBeInTheDocument()
    })
  })
})
