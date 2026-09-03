"""Quad-camera decoding matching the stack_red_cones converter layout."""

from __future__ import annotations

from io import BytesIO
from threading import Lock

import cv2
import numpy as np
from PIL import Image

try:
    import av
except ImportError:  # pragma: no cover - exercised in deployment environments without PyAV
    av = None


class ImageError(ValueError):
    pass


class H264FramePending(ImageError):
    """The decoder has not received a keyframe or output frame yet."""


class H264Decoder:
    """Decode a sequence of H264 access units while retaining codec state."""

    def __init__(self) -> None:
        self._codec = None
        self._decoded_frame = False
        self._lock = Lock()

    def reset(self) -> None:
        with self._lock:
            self._codec = None
            self._decoded_frame = False

    def decode_rgb(self, encoded: bytes) -> np.ndarray:
        if av is None:
            raise ImageError("H264 input requires the PyAV package; install the deployment dependencies")
        if not encoded:
            raise ImageError("H264 packet is empty")

        with self._lock:
            if self._codec is None:
                self._codec = av.CodecContext.create("h264", "r")
            try:
                frames = self._codec.decode(av.Packet(encoded))
            except Exception as exc:
                # A malformed packet can poison the decoder. The next keyframe
                # can recover only after starting a fresh codec context.
                had_decoded_frame = self._decoded_frame
                self._codec = None
                self._decoded_frame = False
                if not had_decoded_frame:
                    raise H264FramePending(f"H264 decoder is waiting for a keyframe: {exc}") from exc
                raise ImageError(f"cannot decode H264 packet: {exc}") from exc
            if not frames:
                raise H264FramePending("H264 packet yielded no decoded frame; waiting for a keyframe")
            try:
                image = frames[-1].to_ndarray(format="rgb24").copy()
                self._decoded_frame = True
                return image
            except Exception as exc:
                raise ImageError(f"cannot convert decoded H264 frame to RGB: {exc}") from exc


_DEFAULT_H264_DECODER = H264Decoder()


def _is_h264_format(image_format: str | None) -> bool:
    normalized = (image_format or "").strip().lower()
    return "h264" in normalized or "avc" in normalized


def decode_rgb(
    encoded: bytes,
    image_format: str | None = None,
    *,
    h264_decoder: H264Decoder | None = None,
) -> np.ndarray:
    if _is_h264_format(image_format):
        return (h264_decoder or _DEFAULT_H264_DECODER).decode_rgb(encoded)
    try:
        with Image.open(BytesIO(encoded)) as image:
            return np.asarray(image.convert("RGB"), dtype=np.uint8).copy()
    except Exception as exc:
        raise ImageError(f"cannot decode quad image: {exc}") from exc


# Geometry of the live /quad_tile/compressed_undistorted H264 stream, measured on the
# controller (6.6.7.100): a 1920x1488 frame of four 960x744 tiles
# (TL head/left eye, TR right eye unused, BL left wrist, BR right wrist) with a timestamp
# bar burned into the bottom 48 rows across the full width.
# This matches the training-data pipeline: the official recorder downscales the same stream
# to 1920x1080 (960x540 tiles, 35-row footer) and marvinpro_data_trans.convert_marvinpro_pro_bags
# crops cam_high to rows [0, 540) and the wrist tiles to rows [540, 1044) — i.e. full tile
# minus the footer. Do NOT "derive" the tile height from the width: the tiles are not 4:3,
# and a heuristic silently mis-crops (that bug bled 24 rows of the head camera into the
# wrist views). Any other frame size means the camera pipeline changed — fail loudly.
QUAD_WIDTH = 1920
QUAD_HEIGHT = 1488
TILE_WIDTH = QUAD_WIDTH // 2  # 960
TILE_HEIGHT = QUAD_HEIGHT // 2  # 744
FOOTER_TOP_ROW = 1440  # first burned-in timestamp row; wrist content is [TILE_HEIGHT, FOOTER_TOP_ROW)


def split_quad_rgb(image: np.ndarray, output_width: int = 640, output_height: int = 480) -> dict[str, np.ndarray]:
    """Split the 1920x1488 quad into the three training cameras, dropping the timestamp footer."""
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ImageError(f"expected HWC RGB image, got {image.shape}")
    height, width = image.shape[:2]
    if (width, height) != (QUAD_WIDTH, QUAD_HEIGHT):
        raise ImageError(
            f"expected {QUAD_WIDTH}x{QUAD_HEIGHT} quad frame from /quad_tile/compressed_undistorted, "
            f"got {width}x{height} — camera stream geometry changed, update the QUAD/TILE/FOOTER constants"
        )

    crops = {
        "cam_high": image[0:TILE_HEIGHT, 0:TILE_WIDTH],
        "cam_left_wrist": image[TILE_HEIGHT:FOOTER_TOP_ROW, 0:TILE_WIDTH],
        "cam_right_wrist": image[TILE_HEIGHT:FOOTER_TOP_ROW, TILE_WIDTH:QUAD_WIDTH],
    }
    result = {}
    for name, crop in crops.items():
        result[name] = cv2.resize(
            np.ascontiguousarray(crop),
            (output_width, output_height),
            interpolation=cv2.INTER_AREA,
        )
    return result


def decode_and_split(
    encoded: bytes,
    image_format: str | None = None,
    *,
    h264_decoder: H264Decoder | None = None,
) -> dict[str, np.ndarray]:
    return split_quad_rgb(decode_rgb(encoded, image_format, h264_decoder=h264_decoder))
