import sqlite3
import datetime

from .server.src.routes import DB_PATH

refresh_rate = 100
count = 0

def log_location(gps_data):
    global refresh_rate
    global count
    
    count = count + 1
    # Only log the once every refresh_rate triggers
    if count < refresh_rate:
        return
    
    long = gps_data["longitude"]
    lat = gps_data["latitude"]
    
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('INSERT INTO locationlogs (timestamp, latitude, longtitude) VALUES (?)', (datetime.datetime.now(),lat, long))
    conn.commit()
    conn.close()
    print(f"Logged values: {lat} {long}")
    
    count = 0
    
