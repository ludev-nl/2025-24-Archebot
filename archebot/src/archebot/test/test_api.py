import pytest
import json
import io

import sys
import os
# Adjust path to find Flask app
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/server/src')))

from routes import app  # Import Flask app

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))  

@pytest.fixture
def client():
    with app.test_client() as client:
        yield client

def test_get_location_logs(client):
    """Test retrieving location logs"""
    response = client.get('/locationlogs')
    assert response.status_code == 200
    data = response.get_json()
    assert isinstance(data, list)

def test_add_location_log(client):
    """Test adding a location log"""
    
    # Correct payload
    payload = {
        "timestamp": "2025-03-13T14:10:00Z",
        "latitude": 51.9225,
        "longitude": 4.47917
    }
    response = client.post('/locationlogs', data=json.dumps(payload), content_type='application/json')
    assert response.status_code == 201
    data = response.get_json()
    assert "message" in data
    
    # Incorrect payload
    payload = {
        "latitude": 51.9225,
        "longitude": 4.47917
    }
    response = client.post('/locationlogs', data=json.dumps(payload), content_type='application/json')
    assert response.status_code == 400
    data = response.get_json()
    assert "error" in data
    

def test_get_logs(client):
    """Test retrieving logs"""
    response = client.get('/logs')
    assert response.status_code == 200
    data = response.get_json()
    assert isinstance(data, list)

def test_add_log(client):
    """Test adding a log"""
    # Correct payload
    payload = {
        "message": "Test log entry",
        "timestamp": "2025-03-13T14:10:00Z"
    }
    response = client.post('/logs', data=json.dumps(payload), content_type='application/json')
    assert response.status_code == 201
    data = response.get_json()
    assert "message" in data
    
    # Incorrect payload
    payload = {
        "message": "Test log entry",
    }
    response = client.post('/logs', data=json.dumps(payload), content_type='application/json')
    assert response.status_code == 400
    data = response.get_json()
    assert "error" in data

def test_get_shards(client):
    """Test retrieving shards"""
    response = client.get('/shards')
    assert response.status_code == 200
    data = response.get_json()
    assert isinstance(data, list)
    if data:
        assert "id" in data[0]
        assert "latitude" in data[0]
        assert "longitude" in data[0]
        assert "photo" in data[0]

def test_add_shard(client):
    """Test adding a shard"""
    # Correct payload
    payload = {
        "latitude": 51.9225,
        "longitude": 4.47917,
        "photo": "dGVzdGltYWdl"  # Base64 encoded test string
    }
    response = client.post('/shards', data=json.dumps(payload), content_type='application/json')
    assert response.status_code == 201
    data = response.get_json()
    assert "message" in data or "error" in data
    
    # Incorrect payload
    payload = {
        "latitude": 51.9225,
        "photo": "dGVzdGltYWdl"  # Base64 encoded test string
    }
    response = client.post('/shards', data=json.dumps(payload), content_type='application/json')
    assert response.status_code == 400
    data = response.get_json()
    assert "error" in data

def test_upload_gpx_file(client):
    """Test uploading a valid GPX file"""
    filename = "test_route.gpx"
    file_content = io.BytesIO(b"<?xml version='1.0' encoding='UTF-8'?><gpx></gpx>")  # Simulate GPX file

    response = client.post(
        f'/routes/{filename}',
        data={'file': (file_content, filename)},
        content_type='multipart/form-data'
    )

    assert response.status_code == 201

def test_get_gpx_file(client):
    """Test retrieving a valid GPX file"""
    filename = "test_route.gpx"
    
    response = client.get(f'/routes/{filename}')
    
    if os.path.exists(os.path.join(SCRIPT_DIR, "../src/server/db/routes", filename)):
        assert response.status_code == 200
    else:
        assert response.status_code == 404