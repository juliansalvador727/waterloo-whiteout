from __future__ import annotations

import importlib.util
import unittest
from datetime import datetime, timezone

from whiteout.camera import CameraFrame


@unittest.skipUnless(
    importlib.util.find_spec("cv2") and importlib.util.find_spec("numpy"),
    "camera decoding dependencies are not installed",
)
class CameraFrameTests(unittest.TestCase):
    def test_decode_crops_only_the_right_edge(self) -> None:
        import cv2
        import numpy as np

        image = np.zeros((720, 960, 3), dtype=np.uint8)
        image[:, 919:] = (255, 255, 255)
        encoded, jpeg = cv2.imencode(".jpg", image)
        self.assertTrue(encoded)
        frame = CameraFrame(
            "quadcopter",
            jpeg.tobytes(),
            datetime.now(timezone.utc),
            crop_right_px=40,
        )

        decoded = frame.decode_bgr()

        self.assertEqual(decoded.shape, (720, 920, 3))

    def test_decode_rejects_crop_that_removes_entire_frame(self) -> None:
        import cv2
        import numpy as np

        encoded, jpeg = cv2.imencode(".jpg", np.zeros((2, 3, 3), dtype=np.uint8))
        self.assertTrue(encoded)
        frame = CameraFrame(
            "quadcopter",
            jpeg.tobytes(),
            datetime.now(timezone.utc),
            crop_right_px=3,
        )

        with self.assertRaisesRegex(ValueError, "invalid right crop"):
            frame.decode_bgr()


if __name__ == "__main__":
    unittest.main()
