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

def test_get_location_log(client):
    """Test retrieving location logs"""
    response = client.get('/locationlog')
    assert response.status_code == 200

def test_get_logs(client):
    """Test retrieving logs"""
    response = client.get('/logs')
    assert response.status_code == 200
    data = response.get_json()
    assert isinstance(data, list)

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