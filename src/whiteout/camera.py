"""Resilient MJPEG ingestion without implicit network activity on import."""

from __future__ import annotations

import time
import urllib.request
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from datetime import datetime, timezone


@dataclass(frozen=True, slots=True)
class CameraFrame:
    camera: str
    jpeg: bytes
    timestamp: datetime

    def decode_bgr(self):
        """Decode with OpenCV when its optional dependency is installed."""
        try:
            import cv2  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("frame decoding requires whiteout[camera]") from exc
        image = cv2.imdecode(np.frombuffer(self.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("invalid JPEG frame")
        return image


class MjpegCamera:
    def __init__(
        self,
        name: str,
        url: str,
        *,
        reconnect_delay_s: float = 1.0,
        timeout_s: float = 5.0,
        opener: Callable[..., object] = urllib.request.urlopen,
    ) -> None:
        self.name = name
        self.url = url
        self.reconnect_delay_s = reconnect_delay_s
        self.timeout_s = timeout_s
        self._opener = opener

    def frames(self, *, reconnect: bool = True) -> Iterator[CameraFrame]:
        while True:
            try:
                with self._opener(self.url, timeout=self.timeout_s) as response:  # type: ignore[attr-defined]
                    yield from self._read_stream(response)
            except (OSError, TimeoutError, ValueError):
                if not reconnect:
                    return
                time.sleep(self.reconnect_delay_s)

    def _read_stream(self, response: object) -> Iterator[CameraFrame]:
        buffer = bytearray()
        while True:
            chunk = response.read(4096)  # type: ignore[attr-defined]
            if not chunk:
                raise OSError("MJPEG stream ended")
            buffer.extend(chunk)
            while True:
                start = buffer.find(b"\xff\xd8")
                end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                if start < 0 or end < 0:
                    break
                jpeg = bytes(buffer[start : end + 2])
                del buffer[: end + 2]
                yield CameraFrame(self.name, jpeg, datetime.now(timezone.utc))
            if len(buffer) > 8_000_000:
                del buffer[:-2]
