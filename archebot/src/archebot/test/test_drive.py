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
    ]

    driver.read_gpx_file()

    assert expected == driver.coordinate_list, "coordinates were not read correctly"
