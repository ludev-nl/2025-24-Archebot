
# API Documentation

This document provides an overview of the API endpoints, their purposes, the data they expect, and the responses they provide.

---

## Base URL
The API is hosted on the Leorover: `http://10.0.0.1:5000`

---

## Endpoints

### 1. **Location Logs**

#### **GET** `/locationlogs`
- **Description:** Retrieve all location logs from the Leorover.
- **Response:**
    - **200 OK:** A list of location logs in the following format:
    ```json
    [
      {
        "timestamp": "2025-03-13T14:10:00Z",
        "latitude": 51.9225,
        "longitude": 4.47917
      }
    ]
    ```
    - **500 Internal Server Error:** If the database query fails.

---

### 2. **Logs**

#### **GET** `/logs`
- **Description:** Retrieve all logs.
- **Response:**
    - **200 OK:** A list of logs in the following format:
    ```json
    [
      {
        "message": "Some log message",
        "timestamp": "2025-03-13T14:10:00Z"
      }
    ]
    ```

---

### 3. **Shards**

#### **GET** `/shards`
- **Description:** Retrieve all shards, including their photos as url. Can be retrieved using `\static\photo_url`.
- **Response:**
    - **200 OK:** A list of shards in the following format:
    ```json
    [
      {
        "id": 1,
        "latitude": 51.9225,
        "longitude": 4.47917,
        "photo": "photo_url"
      }
    ]
    ```
    - **500 Internal Server Error:** If the database query fails.

---

### 4. **Routes**

#### **GET** `/routes`
- **Description:** Get the names of all routes.
- **Response:**
    - **200 OK:** A list of routes in the following format:
      ```json
      {
        "gpx_files": [
          "test_route.gpx"
          ]
      }```
    - **404 Unauthorized:** If the routes directory does not exist.

---

#### **GET** `/routes/<filename>`
- **Description:** Returns the file specified. Must have a .gpx file extention.
- **Response:**
    - **200 OK:** The .gpx file specified in the request.
    - **404 Page Not Found:** The file does not exist or is not valid.

---

#### **POST** `/routes/<filename>`
- **Description:** Save a file to the server. Must be a .gpx file. Request must be of type `multipart/form-data` with key `file` and value the actual .gpx file. 
- **Response:** 
    - **201 OK:** The .gpx file was succesfully saved.
    - **400 Bad Request:** Validation error.

---

#### **GET** `/box-coordinates`
- **Description** Given four coordinates, returns a lawnmower path. 
  ```json
    [
      {
        "southWest": {"lat": 1.23, "lng": 4.56},
        "southEast": {"lat": 1.23, "lng": 4.56},
        "northWest": {"lat": 1.23, "lng": 4.56},
        "northEast": {"lat": 1.23, "lng": 4.56}
      }
    ]
  ```
- **Response** A list of coordinates, forming the lawnmower path. 
  ```json
    [
      {
        "coordinates": [ [1.23, 4.56], [7.89, 10.11] ]
      }
    ]
  ```

---

### 5. **Events**

#### **POST** `/start`
- **Description** Give the Leorover the start command.
- **Response**
    - **200 OK:** Command received.

## Notes
- The API uses Flask as framework for the webserver.
- The API uses Marshmallow schemas for data validation.