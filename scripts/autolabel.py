#!/usr/bin/env python3
"""Pre-label recorded frames as a YOLO boat/ice dataset for manual review."""

from __future__ import annotations

import argparse
import random
import re
from pathlib import Path

import cv2
import numpy as np
import torch
from torchvision.ops import nms
from ultralytics import YOLO

BOAT, ICE = 0, 1


def tiles(width: int, height: int, size: int = 640, overlap: float = 0.25) -> list[tuple[int, int]]:
    step = int(size * (1 - overlap))

    def starts(length: int) -> list[int]:
        if length <= size:
            return [0]
        values = list(range(0, length - size, step))
        return values + [length - size]

    return [(x, y) for y in starts(height) for x in starts(width)]


def boat_boxes(model: YOLO, image: np.ndarray, conf: float) -> list[tuple[float, float, float, float]]:
    height, width = image.shape[:2]
    origins = tiles(width, height)
    crops = [image[y : y + 640, x : x + 640] for x, y in origins]
    # Boat is class 8 in COCO weights and class 0 in our fine-tuned weights.
    boat_id = next(index for index, name in model.names.items() if name == "boat")
    results = model.predict(crops, imgsz=1280, conf=conf, classes=[boat_id], verbose=False)
    boxes, scores = [], []
    for (x, y), result in zip(origins, results, strict=True):
        for (x1, y1, x2, y2), score in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), strict=True):
            boxes.append((x1 + x, y1 + y, x2 + x, y2 + y))
            scores.append(score)
    if not boxes:
        return []
    keep = nms(torch.tensor(boxes), torch.tensor(scores), 0.5).tolist()
    return [boxes[i] for i in keep]


def ice_boxes(image: np.ndarray, min_area: int) -> list[tuple[float, float, float, float]]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 2] > 190) & (hsv[..., 1] < 35)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        if cv2.contourArea(contour) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        boxes.append((x, y, x + w, y + h))
    return boxes


def red_hull_box(image: np.ndarray, min_pixels: int = 4) -> tuple[float, float, float, float] | None:
    """Box around the largest red blob (the boat's hull), padded to cover the superstructure."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = (((hue <= 8) | (hue >= 170)) & (sat > 90) & (val > 60)).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(cv2.dilate(red, np.ones((5, 5), np.uint8)))
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

    # Grow to the whole ship: the non-dark region touching the red blob, within a local window.
    reach = max(30, 3 * max(w, h))
    wx1, wy1 = max(0, x - reach), max(0, y - reach)
    wx2, wy2 = min(width, x + w + reach), min(height, y + h + reach)
    solid = ((val[wy1:wy2, wx1:wx2] > 70) | (labels[wy1:wy2, wx1:wx2] == best)).astype(np.uint8)
    solid = cv2.morphologyEx(solid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _, parts, part_stats, _ = cv2.connectedComponentsWithStats(solid)
    seed = parts[labels[wy1:wy2, wx1:wx2] == best]
    seed = seed[seed > 0]
    if seed.size == 0:
        return fallback
    px, py, pw, ph = part_stats[np.bincount(seed).argmax()][:4]
    touches_edge = px == 0 or py == 0 or px + pw >= wx2 - wx1 or py + ph >= wy2 - wy1
    if touches_edge or pw * ph > 0.6 * (wx2 - wx1) * (wy2 - wy1):
        return fallback  # merged with ice, land, sky or bright water streaks
    return (wx1 + px, wy1 + py, wx1 + px + pw, wy1 + py + ph)


def overlaps(a: tuple[float, ...], b: tuple[float, ...]) -> bool:
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def yolo_line(cls: int, box: tuple[float, ...], width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    return (
        f"{cls} {(x1 + x2) / 2 / width:.6f} {(y1 + y2) / 2 / height:.6f} "
        f"{(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="*", type=Path, default=[Path("recordings")])
    parser.add_argument("--out", type=Path, default=Path("dataset"))
    parser.add_argument("--every", type=int, default=10, help="keep every Nth frame per camera")
    parser.add_argument("--boat-conf", type=float, default=0.3)
    parser.add_argument("--ice-min-area", type=int, default=40)
    parser.add_argument("--val-fraction", type=float, default=0.15)
    parser.add_argument("--weights", default="yolov8s.pt")
    parser.add_argument("--preview", action="store_true", help="also write annotated copies to OUT/preview")
    parser.add_argument(
        "--boat-in-every-frame",
        action="store_true",
        help="fall back to the red hull when YOLO finds no boat; skip frames where neither finds one",
    )
    args = parser.parse_args()
    if args.every < 1:
        parser.error("--every must be positive")

    frames: list[Path] = []
    for source in args.inputs:
        found = sorted(source.rglob("*.jpg")) + sorted(source.rglob("*.png")) if source.is_dir() else [source]
        frames.extend(path for path in found if "annotated" not in path.parts)
    frames = [path for index, path in enumerate(frames) if index % args.every == 0]
    if not frames:
        parser.error("no frames found")

    rng = random.Random(0)
    model = YOLO(args.weights)
    for split in ("train", "val"):
        (args.out / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.out / "labels" / split).mkdir(parents=True, exist_ok=True)
    if args.preview:
        (args.out / "preview").mkdir(parents=True, exist_ok=True)

    boat_total = ice_total = red_total = 0
    skipped: list[str] = []
    for path in frames:
        image = cv2.imread(str(path))
        if image is None:
            continue
        height, width = image.shape[:2]
        boats = boat_boxes(model, image, args.boat_conf)
        red_boats = []
        if not boats and args.boat_in_every_frame:
            hull = red_hull_box(image)
            if hull is None:
                skipped.append(path.name)
                continue
            red_boats = [hull]
        boats += red_boats
        ice = [box for box in ice_boxes(image, args.ice_min_area) if not any(overlaps(box, b) for b in boats)]
        boat_total += len(boats)
        red_total += len(red_boats)
        ice_total += len(ice)

        split = "val" if rng.random() < args.val_fraction else "train"
        stem = re.sub(r"[^\w.-]+", "_", f"{path.parent.name}-{path.stem}")
        cv2.imwrite(str(args.out / "images" / split / f"{stem}.jpg"), image)
        lines = [yolo_line(BOAT, box, width, height) for box in boats]
        lines += [yolo_line(ICE, box, width, height) for box in ice]
        (args.out / "labels" / split / f"{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))

        if args.preview:
            for box in ice:
                cv2.rectangle(image, tuple(map(int, box[:2])), tuple(map(int, box[2:])), (255, 255, 0), 1)
            for box in boats:
                colour = (0, 255, 0) if box in red_boats else (0, 0, 255)
                cv2.rectangle(image, tuple(map(int, box[:2])), tuple(map(int, box[2:])), colour, 2)
            cv2.imwrite(str(args.out / "preview" / f"{stem}.jpg"), image)

    (args.out / "data.yaml").write_text(
        f"path: {args.out.resolve()}\ntrain: images/train\nval: images/val\nnames:\n  0: boat\n  1: ice\n"
    )
    print(
        f"{len(frames) - len(skipped)} frames, {boat_total} boat boxes "
        f"({red_total} from red hull), {ice_total} ice boxes -> {args.out}"
    )
    if skipped:
        print(f"skipped {len(skipped)} frames with no boat found:")
        print("\n".join(f"  {name}" for name in skipped))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
