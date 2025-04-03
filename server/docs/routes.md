
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

#### **POST** `/locationlogs`
- **Description:** Add a new location log.
- **Request:**
  - JSON payload in the following format:
    ```json
    {
      "timestamp": "2025-03-13T14:10:00Z",
      "latitude": 51.9225,
      "longitude": 4.47917
    }
    ```
- **Response:**
  - **201 Created:** Success message:
    ```json
    { "message": "Location log added" }
    ```
  - **400 Bad Request:** Validation errors.
  - **500 Internal Server Error:** Any other errors.

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

#### **POST** `/logs`
- **Description:** Add a new log.
- **Request:**
  - JSON payload in the following format:
    ```json
    {
      "message": "Some log message",
      "timestamp": "2025-03-13T14:10:00Z"
    }
    ```
- **Response:**
  - **201 Created:** Success message:
    ```json
    { "message": "Log added" }
    ```
  - **400 Bad Request:** Validation errors.
  - **500 Internal Server Error:** Any other errors.

---

### 3. **Shards**

#### **GET** `/shards`
- **Description:** Retrieve all shards, including their photos as base64-encoded strings.
- **Response:**
  - **200 OK:** A list of shards in the following format:
    ```json
    [
      {
        "id": 1,
        "latitude": 51.9225,
        "longitude": 4.47917,
        "photo": "base64encodedstring"
      }
    ]
    ```
  - **500 Internal Server Error:** If the database query fails.

---

#### **POST** `/shards`
- **Description:** Add a new shard.
- **Request:**
  - JSON payload in the following format:
    ```json
    {
      "latitude": 51.9225,
      "longitude": 4.47917,
      "photo": "base64encodedstring"
    }
    ```
- **Response:**
  - **201 Created:** Success message:
    ```json
    { "message": "Shard added" }
    ```
  - **400 Bad Request:** Validation errors.
  - **500 Internal Server Error:** Any other errors.

---

## 4. **Routes**

#### **GET** `/routes`
- **Description:** Get the names of all routes.
- **Response:**
  - **200 OK:** A list of routes in the following format:
    ```json
    {
      "gpx_files": [
        "test_route.gpx"
        ]
    }

  - **404 Unauthorized:** If the routes directory does not exist.

#### **GET** `/routes/<filename>`
- **Description:** Returns the file specified. Must have a .gpx file extention.
- **Response:**
  - **200 OK:** The .gpx file specified in the request.
  - **404 Page Not Found:** The file does not exist or is not valid.

#### **POST** `/routes/<filename>`
- **Description:** Save a file to the server. Must be a .gpx file.
- **Response:** 
  - **201 OK:** The .gpx file was succesfully saved.
  - **400 Bad Request:** Validation error.

---

## Notes
- The API uses Flask as framework for the webserver.
- The API uses Marshmallow schemas for data validation.