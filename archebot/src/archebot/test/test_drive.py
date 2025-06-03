import math

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

    driver.read_gpx_file()

    assert expected == driver.coordinate_list, "coordinates were not read correctly"


def test_update_gps_target():
    driver = Driver()

    # first read gpx file
    driver.read_gpx_file()
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

    expected = math.atan2(2 * (w * z + x * y), 1 - 2 * (x * x + y * y))

    driver.update_imu(fake_imu_data)

    assert driver.imu_yaw == expected, (
        "yaw was not calculated correctly from quaternion"
    )
