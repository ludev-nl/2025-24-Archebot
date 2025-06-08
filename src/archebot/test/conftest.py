import pytest
import sys
import os

# Add src symlink to python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src/server/db")))

from init_db import initialize_db

# Before running tests, init database if it does not exist
@pytest.fixture(scope="session", autouse=True)
def setup_database():
    initialize_db()
