from fractions import Fraction
from io import BytesIO
import unittest

import numpy as np
from PIL import Image

try:
    import av
except ImportError:  # pragma: no cover - depends on the test environment
    av = None

from marvinpro_deploy.image_processing import H264Decoder, H264FramePending, decode_and_split, split_quad_rgb


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

    @unittest.skipIf(av is None, "PyAV is not installed")
    def test_h264_decode_keeps_codec_state_between_packets(self):
        encoder = av.CodecContext.create("h264", "w")
        encoder.width = 128
        encoder.height = 96
        encoder.pix_fmt = "yuv420p"
        encoder.time_base = Fraction(1, 30)
        encoder.framerate = Fraction(30, 1)
        encoder.options = {
            "preset": "ultrafast",
            "tune": "zerolatency",
            "repeat-headers": "1",
            "g": "1",
        }

        packets = []
        for index in range(2):
            frame = self.make_quad()[:96, :128].copy()
            frame[:48, :64] = (10 + index * 40, 20, 30)
            for packet in encoder.encode(av.VideoFrame.from_ndarray(frame, format="rgb24")):
                packets.append(bytes(packet))

        decoder = H264Decoder()
        first = decode_and_split(packets[0], "h264", h264_decoder=decoder)
        second = decode_and_split(packets[1], "h264", h264_decoder=decoder)
        self.assertEqual(first["cam_high"].shape, (480, 640, 3))
        np.testing.assert_allclose(first["cam_high"][100, 100], (10, 20, 30), atol=5)
        np.testing.assert_allclose(second["cam_high"][100, 100], (50, 20, 30), atol=5)

    @unittest.skipIf(av is None, "PyAV is not installed")
    def test_h264_decoder_marks_pre_keyframe_packet_as_pending(self):
        with self.assertRaises(H264FramePending):
            H264Decoder().decode_rgb(b"not-a-keyframe")


if __name__ == "__main__":
    unittest.main()
