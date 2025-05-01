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

NDEBUG = True
if not NDEBUG:
    result_count = 0

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
    try:
        os.mkdir("archebot")
    except Exception:
        pass
    # shard_image.save(f"archebot/{data.longitude},{data.latitude}.jpg")
    # database entry
    conn = get_db_connection()
    conn.execute('INSERT INTO shards (latitude, longitude, photo) VALUES (?, ?, ?)', (data.latitude, data.longitude, sqlite3.Binary(shard_image)))
    conn.commit()
    conn.close()
    print("database")
    shard_image = None

def shard_detection(data) -> None:
    global camera_iter
    camera_iter += 1
    if camera_iter % 60:
        return
    global shard_image
    if shard_image is None:
        data = ros_numpy.numpify(data)
        if NDEBUG:
            results = model.predict(data, stream=True, verbose=False)
        else:
            results = model.predict(data, name="tmp", save=True, stream=True, project="archebot", verbose=True)
        for result in results:
            if not NDEBUG:
                while not os.path.exists("archebot/tmp/image0.jpg"):
                    pass
                global result_count
                try:
                    os.makedirs("archebot/debug")
                    os.rename("archebot/tmp/image0.jpg", f"archebot/debug/{result_count}.jpg")
                    os.rmdir("archebot/tmp")
                except Exception:
                    pass
                result_count += 1
            if 0 != len(result.boxes):
                # shard_image = Image.fromarray(data)
                shard_image = data
                break
