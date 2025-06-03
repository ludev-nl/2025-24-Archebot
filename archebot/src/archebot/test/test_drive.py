import math
import os

from geometry_msgs.msg import Pose, Quaternion

from src.drive import Driver


def test_read_gpx_file():
    driver = Driver()

    expected = [
        ("52.16491085489991", "4.464449193221768"),
        ("52.16511606102758", "4.464497118022855"),
        ("52.16511248366583", "4.464537829724046"),
        ("52.164907277538155", "4.464489904922958"),
        ("52.1649037001764", "4.464530616624148"),
        ("52.16510890630408", "4.464578541425236"),
        ("52.16491085489991", "4.464449193221768"),
    ]

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    GPX_PATH = os.path.join(SCRIPT_DIR, "route.gpx")
    driver.read_gpx_file(GPX_PATH)

    assert expected == driver.coordinate_list, "coordinates were not read correctly"


def test_update_gps_target():
    driver = Driver()

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    GPX_PATH = os.path.join(SCRIPT_DIR, "route.gpx")
    # first read gpx file
    driver.read_gpx_file(GPX_PATH)
    # save the original coordinate list
    first = driver.coordinate_list
    driver.update_gps_target()

    assert driver.coordinate_list == first[1:], "coordinates were not updated correctly"
    assert driver.target_lat == first[0][0], "latitude not updated correctly"
    assert driver.target_long == first[0][1], "longitude not updated correctly"


def test_update_imu():
    driver = Driver()

    x = 1.2
    y = 0.2
    z = 2.3
    w = 1.0

    fake_imu_data = Pose()
    fake_imu_data.orientation = Quaternion(x=x, y=y, z=z, w=w)

    expected = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    driver.update_imu(fake_imu_data)

    assert driver.imu_yaw == expected, (
        "yaw was not calculated correctly from quaternion"
    )


def test_compute_distance_and_heading():
    driver = Driver()
    # to left
    lat1, lon1 = (51.759530, 4.940984)
    lat2, lon2 = (51.759530, 4.840984)
    meters = 6883

    dist, head = driver.compute_distance_and_heading(lat1, lon1, lat2, lon2)
    print(dist, head)
    # to down
    lat1, lon1 = (51.759530, 4.940984)
    lat2, lon2 = (51.659530, 4.940984)
    # to right
    lat1, lon1 = (51.759530, 4.940984)
    lat2, lon2 = (51.759530, 5.040984)
    # to up
    lat1, lon1 = (51.759530, 4.940984)
    lat2, lon2 = (51.859530, 4.940984)
