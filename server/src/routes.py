from flask import Flask, request, jsonify
import sqlite3
import os
import base64
from datavalidation import LocationLogsSchema, LogsSchema, ShardsSchema
from marshmallow import ValidationError

app = Flask(__name__)

# Get the directory where the script is located
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
DB_PATH = os.path.join(SCRIPT_DIR, '../db', 'database.db')

def get_db_connection():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

@app.route('/locationlogs', methods=['GET'])
def get_location_logs():
    try:
        conn = get_db_connection()
        logs = conn.execute('SELECT * FROM locationlogs').fetchall()
        conn.close()
        return jsonify([dict(log) for log in logs]), 200
    except:
        return jsonify({"message":"Could not query database"}), 500


@app.route('/locationlogs', methods=['POST'])
def add_location_log():
    
    data = request.json
    
    schema = LocationLogsSchema()

    try:
        validated_data = schema.load(data)  # This will raise an exception if validation fails
    except ValidationError as err:
        return jsonify({"error": err.messages}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
    try:
        conn = get_db_connection()
        conn.execute('INSERT INTO locationlogs (timestamp, latitude, longitude) VALUES (?, ?, ?)',
                    (validated_data['timestamp'], validated_data['latitude'], validated_data['longitude']))
        conn.commit()
        conn.close()
        return jsonify({"message": "Location log added"}), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/logs', methods=['GET'])
def get_logs():
    conn = get_db_connection()
    logs = conn.execute('SELECT * FROM logs').fetchall()
    conn.close()
    return jsonify([dict(log) for log in logs])

@app.route('/logs', methods=['POST'])
def add_log():
    
    data = request.json
    
    schema = LogsSchema()
    
    try:
        validated_data = schema.load(data)  # This will raise an exception if validation fails
    except ValidationError as err:
        return jsonify({"error": err.messages}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
    try:
        conn = get_db_connection()
        conn.execute('INSERT INTO logs (message, timestamp) VALUES (?, ?)',
                    (validated_data['message'], validated_data['timestamp']))
        conn.commit()
        conn.close()
        return jsonify({"message": "Log added"}), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 500 

@app.route('/shards', methods=['GET'])
def get_shards():
    try:
        conn = get_db_connection()
        shards = conn.execute('SELECT id, latitude, longitude, photo FROM shards').fetchall()
        conn.close()
        
        # Convert the photo BLOB to a base64 string for each shard
        result = []
        for shard in shards:
            # Create a dictionary to avoid modifying the sqlite3.Row directly
            shard_dict = dict(shard)
            if shard_dict['photo']:
                shard_dict['photo'] = base64.b64encode(shard_dict['photo']).decode('utf-8')
            result.append(shard_dict)
        
        return jsonify(result), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/shards', methods=['POST'])
def add_shard():
    data = request.json
    
    schema = ShardsSchema()
    
    try:
        validated_data = schema.load(data)  # This will raise an exception if validation fails
    except ValidationError as err:
        return jsonify({"error": err.messages}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
    try:
        # Convert base64 to image data
        image_data = base64.b64decode(validated_data['photo'])
        
        conn = get_db_connection()
        conn.execute('INSERT INTO shards (latitude, longitude, photo) VALUES (?, ?, ?)',
                    (validated_data['latitude'], validated_data['longitude'], image_data))
        conn.commit()
        conn.close()
        return jsonify({"message": "Shard added"}), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
