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

from whiteout.redhull import red_hull_box

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
        if args.boat_in_every_frame:
            # The red hull also recovers distant vessels the model missed while
            # finding a nearer one, so it runs whether or not there are boxes.
            hull = red_hull_box(image)
            if hull is not None and not any(overlaps(hull, box) for box in boats):
                red_boats = [hull]
            if not boats and not red_boats:
                skipped.append(path.name)
                continue
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
