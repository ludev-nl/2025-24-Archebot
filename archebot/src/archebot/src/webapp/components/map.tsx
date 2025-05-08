import * as L from "leaflet";
import { useEffect, useState, useRef } from "react";
import "leaflet/dist/leaflet.css";
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";

interface MapProps {
  onBoxChange: (box: {
    southWest: L.LatLng | null;
    northEast: L.LatLng | null;
    southEast: L.LatLng | null;
    northWest: L.LatLng | null;
  }) => void;
  path?: L.LatLngExpression[];
}

const Map = ({ onBoxChange, path }: MapProps) => {
  const [box, setBox] = useState<{
    southWest: L.LatLng | null;
    northEast: L.LatLng | null;
    southEast: L.LatLng | null;
    northWest: L.LatLng | null;
  }>({
    southWest: null,
    northEast: null,
    southEast: null,
    northWest: null,
  });
  

  const boxRef = useRef(box); // ← store current box
  useEffect(() => {
    boxRef.current = box; // ← keep ref in sync
  }, [box]);

  const mapRef = useRef<L.Map | null>(null);
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const rectangleRef = useRef<L.Rectangle | null>(null);
  const pathLineRef = useRef<L.Polyline | null>(null);

  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) return;

    mapRef.current = L.map(mapContainerRef.current).setView([51.505, -0.09], 16);

    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution:
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    }).addTo(mapRef.current);

    mapRef.current.pm.addControls({
      position: "topleft",
      drawMarker: false,
      drawCircleMarker: false,
      drawPolyline: false,
      drawRectangle: true,
      drawPolygon: false,
      drawCircle: false,
      editMode: true,
      dragMode: true,
      rotateMode: true,
      cutPolygon: false,
      removalMode: true,
    });

    mapRef.current.on("pm:create", (e) => {
      if (e.shape === "Rectangle") {
        if (rectangleRef.current) {
          mapRef.current?.removeLayer(rectangleRef.current);
        }

        const layer = e.layer as L.Rectangle;
        rectangleRef.current = layer;
        updateBoxCoordinates(layer);

        layer.on("pm:editend pm:rotateend pm:dragend", () => {
          updateBoxCoordinates(layer);
        });

        layer.on("pm:rotate", () => {
          const rotationCenter = layer.pm.getRotationCenter();
          const angle = layer.pm.getAngle();
          pathLineRef.current?.pm.setRotationCenter(rotationCenter);
          pathLineRef.current?.pm.rotateLayerToAngle(angle);
        });

        layer.on("pm:remove", () => {
          rectangleRef.current = null;
          const emptyBox = {
            southWest: null,
            northEast: null,
            southEast: null,
            northWest: null,
          };
          setBox(emptyBox);
          onBoxChange(emptyBox);

          if (pathLineRef.current) {
            mapRef.current?.removeLayer(pathLineRef.current);
            pathLineRef.current = null;
          }
        });
      }
    });

    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
      }
    };
  }, [onBoxChange]);

  useEffect(() => {
    if (!mapRef.current) return;

    if (pathLineRef.current) {
      mapRef.current.removeLayer(pathLineRef.current);
      pathLineRef.current = null;
    }

    if (!path || path.length === 0) return;

    try {
      const polyline = L.polyline(path, {
        color: "red",
        weight: 4,
        opacity: 0.7,
      }).addTo(mapRef.current);

      pathLineRef.current = polyline;
      polyline.pm.setOptions({
        draggable: false,
        allowRotation: false,
        allowEditing: false,
      });
      polyline.pm.disable();

    } catch (error) {
      console.error("Error creating polyline:", error);
    }
  }, [path]);

  const updateBoxCoordinates = (layer: L.Rectangle) => {
    const corners = layer.getLatLngs() as L.LatLng[][];
    
    console.log("Corners:", corners);
    const newBox = {
      northWest: corners[0][0], // Access the first element in the array for NW
      southWest: corners[0][1], // Access the first element in the array for SW
      southEast: corners[0][2], // Access the first element in the array for SE
      northEast: corners[0][3], // Access the first element in the array for NE
    };

    setBox(newBox);
    onBoxChange(newBox);
  };

  return (
    <div style={{ height: "100vh", width: "100%", position: "relative" }}>
      <div ref={mapContainerRef} style={{ height: "100%", width: "100%" }} />
      <div className="corner-coordinates">
        <p>Draw a rectangle on the map</p>
      </div>
    </div>
  );
};

export default Map;
