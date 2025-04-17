// test("placeholder", () => {
//   expect(true).toBe(true)
// })


import { render, screen, fireEvent, waitFor } from "@testing-library/react"
import OfflineTileManager from "@/components/offline-tile-manager"
import "@testing-library/jest-dom"

describe("OfflineTileManager", () => {
  const mockCache = {
    add: jest.fn().mockResolvedValue(undefined),
  }

  const mockCaches = {
    open: jest.fn().mockResolvedValue(mockCache),
  }

  beforeEach(() => {
    // Mock caches API fresh for each test
    Object.defineProperty(window, "caches", {
      value: mockCaches,
      writable: true,
    })

    jest.clearAllMocks()
  })

  test("renders the tile manager", () => {
    render(<OfflineTileManager />)
    expect(screen.getByText("Offline Map Tiles")).toBeInTheDocument()
    expect(screen.getByText(/Cache map tiles for offline use/)).toBeInTheDocument()
    expect(screen.getByRole("button", { name: /Cache Map Tiles/i })).toBeInTheDocument()
  })

  test("shows progress when caching starts", async () => {
    render(<OfflineTileManager />)
    fireEvent.click(screen.getByRole("button", { name: /Cache Map Tiles/i }))

    await waitFor(() =>
      expect(screen.getByText(/Caching \d+ of \d+ tiles/)).toBeInTheDocument()
    )
  })

  test("shows completion message after caching", async () => {
    render(<OfflineTileManager />)
    fireEvent.click(screen.getByRole("button", { name: /Cache Map Tiles/i }))

    await waitFor(() => {
      expect(mockCaches.open).toHaveBeenCalledWith("map-tiles")
    })

    await waitFor(() =>
      expect(screen.getByRole("button", { name: /Tiles Cached/i })).toBeInTheDocument()
    )
  })

  test("handles cache error gracefully", async () => {
    mockCache.add.mockRejectedValueOnce(new Error("Cache error"))

    render(<OfflineTileManager />)
    fireEvent.click(screen.getByRole("button", { name: /Cache Map Tiles/i }))

    await waitFor(() => {
      expect(mockCaches.open).toHaveBeenCalledWith("map-tiles")
    })

    await waitFor(() =>
      expect(screen.getByRole("button", { name: /Tiles Cached/i })).toBeInTheDocument()
    )
  })
})
