from flask import Flask, request, jsonify, redirect, send_from_directory
from flask_cors import CORS
import sqlite3
import os
from data_validation import RouteSchema, BoxCoordinatesSchema
from pathing import create_gpx
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

@app.route('/locationlog', methods=['GET'])
def get_location_log():
    try:
        conn = get_db_connection()
        log = conn.execute('SELECT * FROM locationlogs ORDER BY id DESC LIMIT 1').fetchone()
        conn.close()

        if log is None:
            return jsonify({"message": "No logs found"}), 404

        return jsonify(dict(log)), 200
    except Exception as e:
        print("Error fetching latest log:", e)
        return jsonify({"message": "Could not query database"}), 500

@app.route('/logs', methods=['GET'])
def get_logs():
    conn = get_db_connection()
    logs = conn.execute('SELECT * FROM logs').fetchall()
    conn.close()
    return jsonify([dict(log) for log in logs]), 200

@app.route('/shards', methods=['GET'])
def get_shards():
    try:
        conn = get_db_connection()
        shards = conn.execute('SELECT id, latitude, longitude, photo FROM shards').fetchall()
        conn.close()
        
        return jsonify([dict(s) for s in shards]), 200
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
    
    return send_from_directory(ROUTES_DIR, filename), 200

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
    schema = BoxCoordinatesSchema()
    data = request.json
    
    try:
        validated_data = schema.load(data)
    except ValidationError as err:
        return jsonify({"error": err.messages}), 400
    
    points = []
    for key, value in validated_data.items():
        points.append((value["lat"], value["lng"]))
    
    # Also create gpx for future features
    gpx, list = create_gpx(points, step_size_m=4, list=True)
    
    with open(os.path.join(ROUTES_DIR, "route.gpx"), 'w+') as f:
        f.write(gpx)
        
    return jsonify({"coordinates": list}), 200

@app.route('/start', methods=['POST'])
def start_process():
     # Set the global environment variable
    os.environ["archebot_start"] = "true"

    print("Start event received")
    return jsonify({"message": "Start event received"}), 200
  
# Start server on port 5000    
if __name__ == '__main__':
    conn = get_db_connection()
    conn.commit()
    conn.close()
    app.run(host='0.0.0.0', port=5000, debug=True)
