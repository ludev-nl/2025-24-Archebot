import os

import pytest

from src.drive import Driver


def test_read_gpx_file():
    driver = Driver()

    expected = [
        (52.16511606102758, 4.464497118022855),
        (52.16511248366583, 4.464537829724046),
        (52.164907277538155, 4.464489904922958),
        (52.1649037001764, 4.464530616624148),
        (52.16510890630408, 4.464578541425236),
        (52.16491085489991, 4.464449193221768),
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
    old = driver.coordinate_list.copy()
    driver.update_gps_target()
    assert driver.coordinate_list == old[1:], "coordinates were not updated correctly"
    assert driver.target_lat == old[0][0], "latitude not updated correctly"
    assert driver.target_long == old[0][1], "longitude not updated correctly"

    old = driver.coordinate_list.copy()
    driver.update_gps_target()
    assert driver.coordinate_list == old[1:], "coordinates were not updated correctly"
    assert driver.target_lat == old[0][0], "latitude not updated correctly"
    assert driver.target_long == old[0][1], "longitude not updated correctly"

    old = driver.coordinate_list.copy()
    driver.update_gps_target()
    assert driver.coordinate_list == old[1:], "coordinates were not updated correctly"
    assert driver.target_lat == old[0][0], "latitude not updated correctly"
    assert driver.target_long == old[0][1], "longitude not updated correctly"

    old = driver.coordinate_list.copy()
    driver.update_gps_target()
    assert driver.coordinate_list == old[1:], "coordinates were not updated correctly"
    assert driver.target_lat == old[0][0], "latitude not updated correctly"
    assert driver.target_long == old[0][1], "longitude not updated correctly"

    old = driver.coordinate_list.copy()
    driver.update_gps_target()
    assert driver.coordinate_list == old[1:], "coordinates were not updated correctly"
    assert driver.target_lat == old[0][0], "latitude not updated correctly"
    assert driver.target_long == old[0][1], "longitude not updated correctly"

    old = driver.coordinate_list.copy()
    driver.update_gps_target()
    assert driver.coordinate_list == old[1:], "coordinates were not updated correctly"
    assert driver.target_lat == old[0][0], "latitude not updated correctly"
    assert driver.target_long == old[0][1], "longitude not updated correctly"

    # last target should exit the code with code 0
    old = driver.coordinate_list.copy()
    with pytest.raises(SystemExit) as wrapped_error:
        driver.update_gps_target()

    assert wrapped_error.type == SystemExit
    assert wrapped_error.value.code == 0
