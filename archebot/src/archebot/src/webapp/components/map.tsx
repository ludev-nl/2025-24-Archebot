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

function toPoint(map: L.Map, latlng: L.LatLng) {
  let zoom = map.getMaxZoom();
  if (zoom === Infinity) {
    zoom = map.getZoom();
  }
  return map.project(latlng, zoom);
}

function toLatLng(map: L.Map, point: L.Point) {
  let zoom = map.getMaxZoom();
  if (zoom === Infinity) {
    zoom = map.getZoom();
  }
  return map.unproject(point, zoom);
}

function getRotatedRectangle(A: L.LatLng, B: L.LatLng, rotation: number, map: L.Map) {
  const startPoint = toPoint(map, A);
  const endPoint = toPoint(map, B);
  const theta = (rotation * Math.PI) / 180;
  const cos = Math.cos(theta);
  const sin = Math.sin(theta);

  const width =
    (endPoint.x - startPoint.x) * cos + (endPoint.y - startPoint.y) * sin;
  const height =
    (endPoint.y - startPoint.y) * cos - (endPoint.x - startPoint.x) * sin;
  const x0 = width * cos + startPoint.x;
  const y0 = width * sin + startPoint.y;
  const x1 = -height * sin + startPoint.x;
  const y1 = height * cos + startPoint.y;

  const p0 = toLatLng(map, startPoint);
  const p1 = toLatLng(map, L.point(x0, y0));
  const p2 = toLatLng(map, endPoint);
  const p3 = toLatLng(map, L.point(x1, y1));
  return [p0, p1, p2, p3];
}

function rotateLatLngAroundCenter(
  point: L.LatLng,
  center: L.LatLng,
  angleDeg: number
): L.LatLng {
  const angleRad = (angleDeg * Math.PI) / 180;

  const dx = point.lng - center.lng;
  const dy = point.lat - center.lat;

  const rotatedLng = dx * Math.cos(angleRad) - dy * Math.sin(angleRad);
  const rotatedLat = dx * Math.sin(angleRad) + dy * Math.cos(angleRad);

  return L.latLng(center.lat + rotatedLat, center.lng + rotatedLng);
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
  const [box2, setBox2] = useState<{
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

  const box2Ref = useRef(box2);
  useEffect(() => {
    box2Ref.current = box2;
  }, [box2]);

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

        layer.on("pm:rotatestart", () => {
          setBox2(boxRef.current);
          console.log("Box2:", box2Ref.current);
        });

        layer.on("pm:rotateend", (e) => {
          const angle = layer.pm.getAngle();
          const center = layer.pm.getRotationCenter();
          updateRotateBoxCoordinates(layer, center, angle);
        });

        layer.on("pm:editend gm:dragend", () => {
          console.log("Edit or drag ended");
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

  const updateRotateBoxCoordinates = (layer: L.Rectangle, center: L.LatLng, degrees: number) => {
    console.log("Rotating corners:", box2Ref.current);
    const rotationCenter = layer.pm.getRotationCenter();
    const angle = layer.pm.getAngle();
    layer.pm.setRotationCenter(rotationCenter);
    layer.pm.rotateLayerToAngle(angle);
    console.log("rotated shape:", layer.getLatLngs());
    const corners = layer.getLatLngs() as L.LatLng[][];
    const newBox = {
      northWest: corners[0][0],
      southWest: corners[0][1],
      southEast: corners[0][2],
      northEast: corners[0][3],
    };
    L.marker(corners[0][0]).addTo(mapRef.current!);
    L.marker(corners[0][1]).addTo(mapRef.current!);
    L.marker(corners[0][2]).addTo(mapRef.current!);
    L.marker(corners[0][3]).addTo(mapRef.current!);
    setBox(newBox);
    onBoxChange(newBox);
  }

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
