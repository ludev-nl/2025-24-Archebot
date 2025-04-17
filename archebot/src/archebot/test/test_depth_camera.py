import random
from src.depth_camera import cam_to_array

class FakeImage:
    def __init__(self):
        self.data = bytes([random.randint(0, 255) for _ in range(614400)])
        self.width = 640
        self.height = 480

def test_cam_to_array():
    fake_data = FakeImage()
    array = cam_to_array(fake_data)
    
    for height in range(fake_data.height):
        for width in range(fake_data.width):
            raw_pixel = int.from_bytes(
                (
                    fake_data.data[
                        (((height * fake_data.width) + width) * 2) : (((height * fake_data.width) + width) * 2) + 2
                    ]
                ),
                byteorder="little",
            )
            array_pixel = array[height][width]

            assert raw_pixel == array_pixel