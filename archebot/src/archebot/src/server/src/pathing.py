import gpxpy
import gpxpy.gpx
import numpy as np
import geopy.distance
from geopy.distance import geodesic
from pyproj import Transformer

def create_gpx(points, step_size_m=1, list=False):
    # Creating a new file:
    gpx = gpxpy.gpx.GPX()
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    
    # Generate path
    list, gpx_segment = generate_lawnmower_path(points, gpx_segment, step_size_m)
    
    # Add path to file
    gpx_track.segments.append(gpx_segment)
    
    if list:
        return gpx.to_xml(), list
    
    # Return file as string
    return gpx.to_xml()

def sort_points(points: list):
    # Create np array from points and calculate the centroid
    # of the rectangle
    points = np.array(points)
    centroid = np.mean(points, axis=0)
    
    # Sort the points based on the angle of the points with 
    # repect to the centroid
    sorted_points = sorted(range(4), key=lambda i: angle(points[i], centroid))
    return [tuple(points[i]) for i in sorted_points]
    
def angle(point: tuple, centroid: tuple):
    vector = point - centroid
    return np.arctan2(vector[1], vector[0])

def generate_lawnmower_path(coordinates, gpx_segment, step_size_m):
    if len(coordinates) != 4:
        raise ValueError("Exactly 4 corners required.")

    # Convert lat/lon to local Cartesian (meters)
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    inverse_transformer = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

    coords_xy = [np.array(transformer.transform(lon, lat)) for lat, lon in coordinates]

    # Sort corners if needed
    p0, p1, p2, p3 = coords_xy

    # Define sweep direction (from p0 to p1) and offset direction (p0 to p3)
    sweep_dir = p1 - p0
    offset_dir = p3 - p0

    # Normalize directions
    sweep_unit = sweep_dir / np.linalg.norm(sweep_dir)
    offset_unit = offset_dir / np.linalg.norm(offset_dir)

    width = np.linalg.norm(offset_dir)
    num_steps = int(np.ceil(width / step_size_m))

    path = []

    for i in range(num_steps + 1):
        offset_vector = offset_unit * (i * step_size_m)
        start = p0 + offset_vector
        end = p1 + offset_vector

        # Alternate direction for each sweep
        if i % 2 == 0:
            line = [start, end]
        else:
            line = [end, start]

        for point in line:
            lon, lat = inverse_transformer.transform(point[0], point[1])
            gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon))
            path.append((lat, lon))

    return path, gpx_segment