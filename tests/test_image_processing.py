from fractions import Fraction
from io import BytesIO
import unittest

import numpy as np
from PIL import Image

try:
    import av
except ImportError:  # pragma: no cover - depends on the test environment
    av = None

from marvinpro_deploy.image_processing import (
    FOOTER_TOP_ROW,
    QUAD_HEIGHT,
    QUAD_WIDTH,
    TILE_HEIGHT,
    TILE_WIDTH,
    H264Decoder,
    H264FramePending,
    ImageError,
    decode_and_split,
    split_quad_rgb,
)

# Tile colors double as contamination markers.
TL = (10, 20, 30)
TR = (40, 50, 60)
BL = (70, 80, 90)
BR = (100, 110, 120)
FOOTER = (200, 210, 220)


class ImageProcessingTest(unittest.TestCase):
    def make_quad(self):
        # real /quad_tile/compressed_undistorted geometry: 1920x1488, four 960x744
        # tiles, timestamp bar burned into the bottom 48 rows
        image = np.zeros((QUAD_HEIGHT, QUAD_WIDTH, 3), dtype=np.uint8)
        image[0:TILE_HEIGHT, 0:TILE_WIDTH] = TL
        image[0:TILE_HEIGHT, TILE_WIDTH:] = TR
        image[TILE_HEIGHT:FOOTER_TOP_ROW, 0:TILE_WIDTH] = BL
        image[TILE_HEIGHT:FOOTER_TOP_ROW, TILE_WIDTH:] = BR
        image[FOOTER_TOP_ROW:] = FOOTER
        return image

    def test_training_camera_layout(self):
        result = split_quad_rgb(self.make_quad(), 64, 48)
        self.assertEqual(result["cam_high"].shape, (48, 64, 3))
        for name, color in (("cam_high", TL), ("cam_left_wrist", BL), ("cam_right_wrist", BR)):
            np.testing.assert_array_equal(result[name][0, 0], color)
            np.testing.assert_array_equal(result[name][-1, -1], color)

    def test_wrist_views_exclude_head_strip_and_timestamp_footer(self):
        # Regression: the old 4:3-tile heuristic bled 24 rows of the head camera into
        # the top of the wrist views and silently accepted the size mismatch.
        result = split_quad_rgb(self.make_quad())
        for name in ("cam_left_wrist", "cam_right_wrist"):
            frame = result[name]
            for contaminant in (TL, TR, FOOTER):
                self.assertFalse(
                    np.any(np.all(frame == contaminant, axis=-1)),
                    f"{name} contains contaminant {contaminant}",
                )

    def test_unexpected_quad_size_fails_loudly(self):
        with self.assertRaises(ImageError):
            split_quad_rgb(np.zeros((1080, 1920, 3), dtype=np.uint8))
        with self.assertRaises(ImageError):
            split_quad_rgb(np.zeros((2160, 3840, 3), dtype=np.uint8))

    def test_jpeg_decode_is_rgb(self):
        source = self.make_quad()
        output = BytesIO()
        Image.fromarray(source).save(output, format="JPEG", quality=95)
        result = decode_and_split(output.getvalue())
        self.assertEqual(result["cam_high"].shape, (480, 640, 3))
        np.testing.assert_allclose(result["cam_high"][100, 100], TL, atol=3)

    @unittest.skipIf(av is None, "PyAV is not installed")
    def test_h264_decode_keeps_codec_state_between_packets(self):
        encoder = av.CodecContext.create("h264", "w")
        encoder.width = QUAD_WIDTH
        encoder.height = QUAD_HEIGHT
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
            frame = self.make_quad()
            frame[0:TILE_HEIGHT, 0:TILE_WIDTH] = (10 + index * 40, 20, 30)
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
