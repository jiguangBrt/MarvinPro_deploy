from io import BytesIO
import unittest

import numpy as np
from PIL import Image

from marvinpro_deploy.image_processing import decode_and_split, split_quad_rgb


class ImageProcessingTest(unittest.TestCase):
    def make_quad(self):
        image = np.zeros((1488, 1920, 3), dtype=np.uint8)
        image[:720, :960] = (10, 20, 30)
        image[:720, 960:] = (40, 50, 60)
        image[720:1440, :960] = (70, 80, 90)
        image[720:1440, 960:] = (100, 110, 120)
        image[1440:] = 255
        return image

    def test_training_camera_layout(self):
        result = split_quad_rgb(self.make_quad(), 64, 48)
        self.assertEqual(tuple(result["cam_high"][0, 0]), (10, 20, 30))
        self.assertEqual(tuple(result["cam_left_wrist"][0, 0]), (70, 80, 90))
        self.assertEqual(tuple(result["cam_right_wrist"][0, 0]), (100, 110, 120))
        self.assertEqual(result["cam_high"].shape, (48, 64, 3))

    def test_jpeg_decode_is_rgb(self):
        source = self.make_quad()
        output = BytesIO()
        Image.fromarray(source).save(output, format="JPEG", quality=95)
        result = decode_and_split(output.getvalue())
        self.assertEqual(result["cam_high"].shape, (480, 640, 3))
        np.testing.assert_allclose(result["cam_high"][100, 100], (10, 20, 30), atol=3)


if __name__ == "__main__":
    unittest.main()
