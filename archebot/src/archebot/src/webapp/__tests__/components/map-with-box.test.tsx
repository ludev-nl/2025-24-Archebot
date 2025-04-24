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

import MapWithBox from "@/components/map-with-box"

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

describe("MapWithBox", () => {
  const mockOnBoxChange = jest.fn()

  beforeEach(() => {
    jest.clearAllMocks()
  })

  test("renders the map container", () => {
    render(<MapWithBox onBoxChange={mockOnBoxChange} />)
    expect(screen.getByTestId("map-container")).toBeInTheDocument()
  })

  test("handles online/offline status changes", async () => {
    render(<MapWithBox onBoxChange={mockOnBoxChange} />)

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
})
