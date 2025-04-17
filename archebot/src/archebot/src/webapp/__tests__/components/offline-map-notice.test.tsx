// import { render, screen, fireEvent } from "@testing-library/react"
// import OfflineMapNotice from "@/components/offline-map-notice"
// import "@testing-library/jest-dom"

// // Mock the navigator.onLine property
// Object.defineProperty(navigator, "onLine", {
//   configurable: true,
//   get: () => true,
// })

// describe("OfflineMapNotice", () => {
//   beforeEach(() => {
//     jest.clearAllMocks()
//   })

//   test("renders online notice when online", () => {
//     render(<OfflineMapNotice />)
//     expect(screen.getByText("Online Mode")).toBeInTheDocument()
//     expect(screen.getByText("You're currently online. The map will use OpenStreetMap tiles.")).toBeInTheDocument()
//   })

//   test("renders offline notice when offline", () => {
//     // Mock navigator.onLine as false
//     Object.defineProperty(navigator, "onLine", {
//       configurable: true,
//       get: () => false,
//     })

//     render(<OfflineMapNotice />)

//     // Trigger the useEffect to update the isOnline state
//     fireEvent(window, new Event("offline"))

//     expect(screen.getByText("Offline Mode")).toBeInTheDocument()
//     expect(
//       screen.getByText("You're currently offline. The map will use locally cached tiles if available."),
//     ).toBeInTheDocument()
//   })

//   test("updates notice when online status changes", () => {
//     render(<OfflineMapNotice />)

//     // Initially online
//     expect(screen.getByText("Online Mode")).toBeInTheDocument()

//     // Trigger offline event
//     Object.defineProperty(navigator, "onLine", {
//       configurable: true,
//       get: () => false,
//     })
//     fireEvent(window, new Event("offline"))

//     // Should show offline notice
//     expect(screen.getByText("Offline Mode")).toBeInTheDocument()

//     // Trigger online event
//     Object.defineProperty(navigator, "onLine", {
//       configurable: true,
//       get: () => true,
//     })
//     fireEvent(window, new Event("online"))

//     // Should show online notice again
//     expect(screen.getByText("Online Mode")).toBeInTheDocument()
//   })
// })
