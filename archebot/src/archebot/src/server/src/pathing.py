import gpxpy.gpx
import numpy as np
import geopy.distance

def create_gpx(points, step_size_m=1):
    # Creating a new file:
    gpx = gpxpy.gpx.GPX()
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    
    # Generate path
    gpx_segment = generate_lawnmower_path(points, gpx_segment, step_size_m)
    
    # Add path to file
    gpx_track.segments.append(gpx_segment)
    
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

    # Sort the corners 
    sorted_points = sort_points(coordinates)
    p0, p1, p2, p3 = [np.array(p) for p in sorted_points]

    # Assume rectangle: p0 -> p1 -> p2 -> p3 -> p0
    # Use edge1 when rotating path    
    # edge1 = p1 - p0  # sweep direction
    edge2 = p3 - p0  # offset direction

    # Calculate the number of steps
    width_m = geopy.distance.distance(p0, p3).m
    num_steps = int(np.ceil(width_m / step_size_m))

    # Get the direction
    # Use dir1 to rotate path 90 degrees
    # dir1 = edge1 / np.linalg.norm(edge1)
    dir2 = edge2 / np.linalg.norm(edge2)
    
    step_size_coord = edge2 / num_steps
    
    for i in range(num_steps + 1):
        offset = dir2 * (i * step_size_coord)
        start = p0 + offset
        end = p1 + offset

        if i % 2 == 0:
            gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(start[0], start[1]))
            gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(end[0], end[1]))
        else:
            gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(end[0], end[1]))
            gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(start[0], start[1]))

    return gpx_segment