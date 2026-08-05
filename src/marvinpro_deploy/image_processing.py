"""Quad-camera decoding matching the stack_red_cones converter layout."""

from __future__ import annotations

from io import BytesIO

import cv2
import numpy as np
from PIL import Image


class ImageError(ValueError):
    pass


def decode_rgb(encoded: bytes) -> np.ndarray:
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


def decode_and_split(encoded: bytes) -> dict[str, np.ndarray]:
    return split_quad_rgb(decode_rgb(encoded))
