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


def split_quad_rgb(image: np.ndarray, output_width: int = 640, output_height: int = 480) -> dict[str, np.ndarray]:
    """Remove the footer and map TL/BL/BR to high/left-wrist/right-wrist."""
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ImageError(f"expected HWC RGB image, got {image.shape}")
    height, width = image.shape[:2]
    if width % 2:
        raise ImageError(f"quad image width must be even, got {width}")
    tile_width = width // 2
    tile_height = int(round(tile_width * 3 / 4))
    canvas_height = 2 * tile_height
    if height < canvas_height:
        raise ImageError(f"quad image {width}x{height} is shorter than camera canvas {width}x{canvas_height}")
    footer_height = height - canvas_height
    if footer_height > max(64, int(0.1 * canvas_height)):
        raise ImageError(f"unexpected quad image footer height: {footer_height}px")

    crops = {
        "cam_high": image[0:tile_height, 0:tile_width],
        "cam_left_wrist": image[tile_height:canvas_height, 0:tile_width],
        "cam_right_wrist": image[tile_height:canvas_height, tile_width:width],
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
