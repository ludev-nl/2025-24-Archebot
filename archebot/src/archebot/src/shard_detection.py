from PIL import Image
from ultralytics import YOLO
import os
import ros_numpy
import sqlite3

cwd = os.path.dirname(os.path.abspath(__file__))
model = YOLO(os.path.join(cwd, "../include/model.pt"))
DB_PATH = os.path.join(cwd, "server/db/database.db")
gps_iter = -1
camera_iter = -1
shard_image = None

def get_db_connection():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def save_location(data) -> None:
    global gps_iter
    gps_iter += 1
    if gps_iter % 60:
        return
    global shard_image, result_count
    if shard_image is None:
        return
    
    filename = f"{data.longitude},{data.latitude}.jpg"
    image_path_relative = f"server/src/static/{filename}"
    index = 1
    # Add _n for images with the same location/no location
    while os.path.exists(cwd + "/" + image_path_relative):
        filename = f"{data.longitude},{data.latitude}_{index}.jpg"
        image_path_relative = f"server/src/static/{data.longitude},{data.latitude}_{index}.jpg"
        index += 1
    
    full_path = f"{cwd}/{image_path_relative}"
    shard_image.save(full_path)
    
    # database entry
    conn = get_db_connection()
    conn.execute('INSERT INTO shards (latitude, longitude, photo) VALUES (?, ?, ?)', (data.latitude, data.longitude, filename))
    conn.commit()
    conn.close()

def shard_detection(data) -> None:
    global camera_iter
    camera_iter += 1
    if camera_iter % 60:
        return
    global shard_image
    if shard_image is None:
        data = ros_numpy.numpify(data)
        results = model.predict(data, stream=True, verbose=False)
        for result in results:
            if 0 != len(result.boxes):
                shard_image = Image.fromarray(data)
                break
