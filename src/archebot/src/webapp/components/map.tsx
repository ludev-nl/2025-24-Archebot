import * as L from "leaflet";
import { useEffect, useState, useRef } from "react";
import "leaflet/dist/leaflet.css";
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";
import type { ShardInfo } from "@/app/page";

const API_IMAGE_URL = process.env.NEXT_PUBLIC_API_BASE_URL + "/static/"
const API_BASE_URL = process.env.NEXT_PUBLIC_API_BASE_URL;

interface MapProps {
  onBoxChange: (box: {
    southWest: L.LatLng | null;
    northEast: L.LatLng | null;
    southEast: L.LatLng | null;
    northWest: L.LatLng | null;
  }) => void;
  path?: L.LatLng[];
  shards?: ShardInfo[];
}

// Map component to display the map and handle rectangle drawing
const Map = ({ onBoxChange, path, shards}: MapProps) => {
  const [latestLog, setLatestLog] = useState<L.LatLng>(); // State to hold the latest location log
  
    // Fetch the latest location log from the server
    useEffect(() => {
      fetch(`${API_BASE_URL}/locationlog`)
        .then((res) => res.json())
        .then((data) => {
          const latestLog = data ? L.latLng(data.latitude, data.longitude) : undefined;
          setLatestLog(latestLog);
        })
        .catch((err) => {
          console.error(err);
        });
    }, []);
    console.log("latest log after loading:", latestLog);  
  
  // State to hold the box coordinates
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
  
  // Ref to store the current box state
  const boxRef = useRef(box);
  useEffect(() => {
    boxRef.current = box;
  }, [box]);


  const mapRef = useRef<L.Map | null>(null); // Ref to store the Leaflet map instance
  const mapContainerRef = useRef<HTMLDivElement>(null); // Ref to the map container element
  const rectangleRef = useRef<L.Rectangle | null>(null); // Ref to store the rectangle layer
  const pathLineRef = useRef<L.Polyline | null>(null); // Ref to store the route layer

  // Function to update the box coordinates when the rectangle is drawn or edited
  const updateBoxCoordinates = (layer: L.Rectangle) => {
    const corners = layer.getLatLngs() as L.LatLng[][];

    console.log("Corners:", corners);
    const newBox = {
      northWest: corners[0][0],
      southWest: corners[0][1],
      southEast: corners[0][2],
      northEast: corners[0][3],
    };

    setBox(newBox);
    onBoxChange(newBox);
  };

  const updateRotateBoxCoordinates = (layer: L.Rectangle) => {
    
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

    setBox(newBox);
    onBoxChange(newBox);
  }

  // Function to handle box creation/changes with geomann and loading the map
  // This effect runs when the map container is ready and the latest log is available
  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) return;

    console.log("Latest log in map componets:", latestLog);
    if (!latestLog) return;
    mapRef.current = L.map(mapContainerRef.current).setView(latestLog, 16);

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
        rectangleRef.current.bringToFront();


        layer.on("pm:rotateend", () => {
          updateRotateBoxCoordinates(layer);
        });

        layer.on("pm:edit pm:dragend", () => {
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
  }, [latestLog, onBoxChange]);

  // Effect to handle the route layer and update it when the path changes
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

      // Disable all Geoman functionality
      polyline.pm.disable();
      polyline.pm.disableRotate();

      pathLineRef.current = polyline;
      pathLineRef.current.pm.setOptions({
        draggable: false,
        allowRotation: false,
        allowEditing: false,
      });
      pathLineRef.current?.bringToBack();

    } catch (error) {
      console.error("Error creating polyline:", error);
    }
  }, [path]);

  // Effect to handle the shards and add markers to the map
  useEffect(() => {
    console.log("shards state:", shards);
    if (!mapRef.current || !shards) return;

    const map = mapRef.current;
    
    console.log("Shards:", shards);
    shards.forEach((shard: ShardInfo) => {
      const shardLatLng = L.latLng(shard.lat, shard.lng);
      let exists = false;

      map.eachLayer((layer) => {
        if (layer instanceof L.Marker) {
          const markerPos = layer.getLatLng();
          if (markerPos.equals(shardLatLng)) {
            exists = true;
            return; // Stop checking if marker already exists
          }
        }
      });

      if (exists) return; // Skip if marker already exists

      if (!shard) return;
      const marker = L.marker(L.latLng(shard.lat, shard.lng), {
        icon: L.icon({
          iconUrl: "/marker.png",
          iconSize: [32, 32], // Width, Height in pixels
          iconAnchor: [16, 16],
        }),
      });

      // Bind a popup with the image
      if (shard.image) {
        marker.bindPopup(`<img src="${API_IMAGE_URL + shard.image}" alt="shard" style="width: 150px;" />`);
      }

      if (!mapRef.current) return;

      marker.addTo(mapRef.current);
    });
  }, [shards])

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
