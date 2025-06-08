import sqlite3
import os

# Get the absolute path to the database file
DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'database.db')

def initialize_db():
    if os.path.exists(DB_PATH):
        print("Database already exists.")
        return
    
    print("Creating database...")
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Create locationlogs table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS locationlogs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME NOT NULL,
            latitude FLOAT NOT NULL,
            longitude FLOAT NOT NULL
        )
    ''')

    # Create logs table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            message VARCHAR(500),
            timestamp DATETIME NOT NULL
        )
    ''')

    # Create shards table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS shards (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            latitude FLOAT NOT NULL,
            longitude FLOAT NOT NULL,
            photo VARCHAR(256)
        )
    ''')

    conn.commit()
    conn.close()
    print("Database initialized successfully.")

if __name__ == "__main__":
    initialize_db()