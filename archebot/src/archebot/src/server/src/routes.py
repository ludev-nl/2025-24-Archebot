from flask import Flask, request, jsonify, redirect, send_from_directory
from flask_cors import CORS
import sqlite3
import os
import base64
from datavalidation import LocationLogsSchema, LogsSchema, ShardsSchema, RouteSchema
from marshmallow import ValidationError
import markdown

app = Flask(__name__)
CORS(app)  # enable CORS globally

# Get the directory where the script is located
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
ROUTES_DIR = os.path.join(SCRIPT_DIR, '../db/routes/')
DB_PATH = os.path.join(SCRIPT_DIR, '../db', 'database.db')

def get_db_connection():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

@app.route('/', methods=['GET'])
def hello():
    return redirect("/docs", code=302)

@app.route('/docs', methods=['GET'])
def get_docs():
    try:
        # Path to the Markdown file
        doc_file_path = os.path.join(os.path.dirname(__file__), '../docs/routes.md')
        
        # Read the Markdown file
        with open(doc_file_path, 'r') as f:
            markdown_content = f.read()
        
        # Convert Markdown to HTML
        html_content = markdown.markdown(markdown_content)
        return html_content, 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

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
    return jsonify([dict(log) for log in logs]), 200

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
    
@app.route('/routes', methods=['GET'])
def get_gpx_files():
    if not os.path.exists(ROUTES_DIR):
        return jsonify({"error": "Routes directory does not exist"}), 404
    
    # Iterate over all files in ROUTES_DIR and return all .gpx files
    gpx_files = [f for f in os.listdir(ROUTES_DIR) if f.endswith('.gpx')]
    return jsonify({"routes": gpx_files}), 200

@app.route('/routes/<filename>', methods=['GET'])
def get_gpx_file(filename):
    if not os.path.exists(ROUTES_DIR):
        return jsonify({"error": "Routes directory does not exist"}), 404
    
    file_path = os.path.join(ROUTES_DIR, filename)
    
    if not os.path.isfile(file_path) or not filename.endswith(".gpx"):
        return jsonify({"error": "File not found or is invalid"}), 404
    
    return send_from_directory(ROUTES_DIR, filename)

@app.route('/routes/<filename>', methods=['POST'])
def upload_gpx_file(filename):
    schema = RouteSchema()
    
    try:
        schema.load(request.files)
    except ValidationError as err:
        return jsonify({"error": err.messages}), 400
    
    if not filename.endswith('.gpx'):
        return jsonify({"error": "Invalid file type, only .gpx files are allowed"}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "No file selected"}), 400

    file_path = os.path.join(ROUTES_DIR, filename)
    file.save(file_path)
    return jsonify({"message": "File uploaded successfully"}), 201

@app.route('/box-coordinates', methods=['POST'])
def receive_box_coordinates():
    try:
        data = request.get_json()
        print("Received box coordinates:", data)

        
        return jsonify({"message": "Box coordinates received"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

  
# Start server on port 5000    
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
