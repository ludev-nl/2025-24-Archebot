"use client"
import { render, screen, fireEvent } from "@testing-library/react"
import "@testing-library/jest-dom"

// Mock the Leaflet and react-leaflet dependencies
jest.mock("leaflet", () => {
  const originalModule = jest.requireActual("leaflet")

  // Mock the Rectangle class
  class MockRectangle {
    getBounds() {
      return {
        getSouthWest: () => ({ lat: 40.7, lng: -74.05 }),
        getNorthEast: () => ({ lat: 40.73, lng: -73.97 }),
      }
    }
  }

  return {
    ...originalModule,
    Rectangle: MockRectangle,
    Icon: {
      Default: {
        prototype: {
          _getIconUrl: jest.fn(),
        },
        mergeOptions: jest.fn(),
      },
    },
    // Mock the map class
    map: jest.fn().mockImplementation(() => ({
      on: jest.fn(),
      addControl: jest.fn(),
      addLayer: jest.fn(),
      remove: jest.fn(),
      invalidateSize: jest.fn(),
      eachLayer: jest.fn(),
    })),
    // Mock the TileLayer class
    TileLayer: jest.fn().mockImplementation(() => ({
      addTo: jest.fn(),
    })),
    // Mock the FeatureGroup class
    FeatureGroup: jest.fn().mockImplementation(() => ({
      addTo: jest.fn(),
      addLayer: jest.fn(),
    })),
    // Mock the Control.Draw class
    Control: {
      Draw: jest.fn().mockImplementation(() => ({
        addTo: jest.fn(),
      })),
    },
  }
})

// Mock the react-leaflet components
jest.mock("react-leaflet", () => ({
  MapContainer: jest.fn(({ children }) => <div data-testid="map-container">{children}</div>),
  TileLayer: jest.fn(() => <div data-testid="tile-layer" />),
  FeatureGroup: jest.fn(({ children }) => <div data-testid="feature-group">{children}</div>),
}))

// Mock the react-leaflet-draw EditControl
jest.mock("react-leaflet-draw", () => ({
  EditControl: jest.fn(({ onCreated, onDeleted, onEdited }) => (
    <div data-testid="edit-control">
      <button
        data-testid="simulate-create"
        onClick={() =>
          onCreated({
            layer: new (jest.requireActual("leaflet").Rectangle)(),
          })
        }
      >
        Simulate Create
      </button>
      <button data-testid="simulate-delete" onClick={() => onDeleted()}>
        Simulate Delete
      </button>
      <button
        data-testid="simulate-edit"
        onClick={() =>
          onEdited({
            layers: {
              eachLayer: (callback: (layer: any) => void) => {
                callback(new (jest.requireActual("leaflet").Rectangle)())
              },
            },
          })
        }
      >
        Simulate Edit
      </button>
    </div>
  )),
}))

// Mock dynamic imports
jest.mock("next/dynamic", () => ({
  __esModule: true,
  default: (fn: any) => {
    const Component = fn()
    Component.displayName = "DynamicComponent"
    return Component
  },
}))

// Mock the navigator.onLine property
Object.defineProperty(navigator, "onLine", {
  configurable: true,
  get: () => true,
})

import MapWithBoundingBox from "@/components/map-with-bounding-box"

describe("MapWithBoundingBox", () => {
  const mockOnBoundingBoxChange = jest.fn()

  beforeEach(() => {
    jest.clearAllMocks()
  })

  test("renders the map container", () => {
    render(<MapWithBoundingBox onBoundingBoxChange={mockOnBoundingBoxChange} />)
    expect(screen.getByTestId("map-container")).toBeInTheDocument()
  })

  test("renders EditControl", async () => {
    render(<MapWithBoundingBox onBoundingBoxChange={mockOnBoundingBoxChange} />)
    
    expect(await screen.findByTestId("edit-control")).toBeInTheDocument()
  })

  test("calls onBoundingBoxChange when a rectangle is created", async () => {
    render(<MapWithBoundingBox onBoundingBoxChange={mockOnBoundingBoxChange} />)

    // Wait for the map to initialize
    await screen.findByTestId("map-container")

    // Simulate the draw:created event via the mocked EditControl button
    fireEvent.click(screen.getByTestId("simulate-create"))

    expect(mockOnBoundingBoxChange).toHaveBeenCalledWith({
      southWest: { lat: 40.7, lng: -74.05 },
      northEast: { lat: 40.73, lng: -73.97 },
    })
  })

  test("calls onBoundingBoxChange with null values when a rectangle is deleted", async () => {
    render(<MapWithBoundingBox onBoundingBoxChange={mockOnBoundingBoxChange} />)

    // Wait for the map to initialize
    await screen.findByTestId("map-container")

    // Simulate the draw:deleted event
    const map = require("leaflet").map.mock.results[0].value
    const deleteHandler = map.on.mock.calls.find((call: any[]) => call[0] === "draw:deleted")[1]

    deleteHandler()

    expect(mockOnBoundingBoxChange).toHaveBeenCalledWith({
      southWest: null,
      northEast: null,
    })
  })

  test("calls onBoundingBoxChange when a rectangle is edited", async () => {
    render(<MapWithBoundingBox onBoundingBoxChange={mockOnBoundingBoxChange} />)

    // Wait for the map to initialize
    await screen.findByTestId("map-container")

    // Simulate the draw:edited event
    const map = require("leaflet").map.mock.results[0].value
    const editHandler = map.on.mock.calls.find((call: any[]) => call[0] === "draw:edited")[1]

    editHandler({
      layers: {
        eachLayer: (callback: (layer: any) => void) => {
          callback({
            getBounds: () => ({
              getSouthWest: () => ({ lat: 40.7, lng: -74.05 }),
              getNorthEast: () => ({ lat: 40.73, lng: -73.97 }),
            }),
          })
        },
      },
    })

    expect(mockOnBoundingBoxChange).toHaveBeenCalledWith({
      southWest: { lat: 40.7, lng: -74.05 },
      northEast: { lat: 40.73, lng: -73.97 },
    })
  })

  test("handles online/offline status changes", async () => {
    render(<MapWithBoundingBox onBoundingBoxChange={mockOnBoundingBoxChange} />)

    // Wait for the map to initialize
    await screen.findByTestId("map-container")

    // Simulate going offline
    Object.defineProperty(navigator, "onLine", {
      configurable: true,
      get: () => false,
    })

    fireEvent(window, new Event("offline"))

    // Check that the tile layer was updated
    const map = require("leaflet").map.mock.results[0].value
    expect(map.eachLayer).toHaveBeenCalled()
  })

  test("provides a reload button for error recovery", async () => {
    render(<MapWithBoundingBox onBoundingBoxChange={mockOnBoundingBoxChange} />)

    // Find the reload button
    const reloadButton = await screen.findByText("Reload Map")
    expect(reloadButton).toBeInTheDocument()

    // Click the reload button
    fireEvent.click(reloadButton)

    // Check that the map was reinitialized
    const map = require("leaflet").map.mock.results[0].value
    expect(map.remove).toHaveBeenCalled()
  })
})
