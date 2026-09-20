"""Colour cue for the vessel's red hull.
"""

from __future__ import annotations


def _red_mask(image):
    import cv2  # type: ignore[import-not-found]
    import numpy as np  # type: ignore[import-not-found]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (((hue <= 8) | (hue >= 170)) & (saturation > 90) & (value > 60)).astype(np.uint8)


def red_fraction(image, box: tuple[float, float, float, float]) -> float:
    """Share of red pixels inside a box, for separating hulls from ice."""
    import numpy as np  # type: ignore[import-not-found]

    height, width = image.shape[:2]
    x1, y1, x2, y2 = (int(round(value)) for value in box)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width, x2), min(height, y2)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    crop = image[y1:y2, x1:x2]
    return float(np.mean(_red_mask(crop)))


def red_hull_box(image, min_pixels: int = 4) -> tuple[float, float, float, float] | None:
    """Box around the largest red blob, grown to the whole hull where possible.

    The red paint is only part of the vessel, so the box is expanded to the
    connected non-water region touching it. That growth is discarded when it
    runs into the search window's edge, which means it merged with ice, land or
    bright water streaks; the padded red blob is returned instead.
    """
    import cv2  # type: ignore[import-not-found]
    import numpy as np  # type: ignore[import-not-found]

    red = _red_mask(image)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        cv2.dilate(red, np.ones((5, 5), np.uint8))
    )
    best, best_pixels = None, min_pixels - 1
    for index in range(1, count):
        pixels = int(red[labels == index].sum())
        if pixels > best_pixels:
            best, best_pixels = index, pixels
    if best is None:
        return None

    x, y, w, h = stats[best][:4]
    height, width = image.shape[:2]
    pad = max(3, int(0.3 * max(w, h)))
    fallback = (max(0, x - pad), max(0, y - pad), min(width, x + w + pad), min(height, y + h + pad))

    reach = max(30, 3 * max(w, h))
    wx1, wy1 = max(0, x - reach), max(0, y - reach)
    wx2, wy2 = min(width, x + w + reach), min(height, y + h + reach)
    value = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[..., 2]
    solid = ((value[wy1:wy2, wx1:wx2] > 70) | (labels[wy1:wy2, wx1:wx2] == best)).astype(np.uint8)
    solid = cv2.morphologyEx(solid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _, parts, part_stats, _ = cv2.connectedComponentsWithStats(solid)
    seed = parts[labels[wy1:wy2, wx1:wx2] == best]
    seed = seed[seed > 0]
    if seed.size == 0:
        return fallback
    px, py, pw, ph = part_stats[np.bincount(seed).argmax()][:4]
    touches_edge = px == 0 or py == 0 or px + pw >= wx2 - wx1 or py + ph >= wy2 - wy1
    if touches_edge or pw * ph > 0.6 * (wx2 - wx1) * (wy2 - wy1):
        return fallback
    return (wx1 + px, wy1 + py, wx1 + px + pw, wy1 + py + ph)
