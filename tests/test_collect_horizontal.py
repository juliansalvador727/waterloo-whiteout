from __future__ import annotations

import json
import math
import threading
import unittest
from datetime import datetime, timezone
from unittest.mock import patch

from whiteout.camera import CameraFrame
from whiteout.control import MavProxyConnectionError
from scripts.collect_horizontal import (
    LatestFrameCamera,
    ModelPose,
    PoseSpec,
    aim_pwm,
    angles_from_pwm,
    automatic_poses,
    models_from_scene,
    public_angles_from_pwm,
    projected_boat_box,
    GazeboBridge,
    start_tower_controller,
    tower_joint_angles_from_scene,
    yaw_quaternion,
    yolo_line,
)


class CollectHorizontalTests(unittest.TestCase):
    def test_automatic_poses_follow_course_and_vary_aspect(self) -> None:
        anchor = ModelPose("target_vessel", 4, 100.0, 200.0, 0.0, math.pi / 2)
        poses = automatic_poses(anchor, 4)

        self.assertEqual(len(poses), 4)
        self.assertLess(poses[0].y, anchor.y)
        self.assertGreater(poses[-1].y, anchor.y)
        self.assertEqual([pose.yaw_deg for pose in poses], [90.0, 180.0, 270.0, 0.0])
        self.assertEqual([pose.view_offset_deg for pose in poses], [-20.0, -7.0, 7.0, 20.0])

    def test_aim_pwm_centres_flat_target_at_expected_values(self) -> None:
        tower = ModelPose("tower-1", 1, 0.0, 0.0, 97.3, 0.0)
        target = PoseSpec("boat", 1000.0, 0.0, 0.0)

        pan_pwm, tilt_pwm, pan, tilt = aim_pwm(tower, target)

        self.assertEqual(pan_pwm, 1500)
        self.assertAlmostEqual(math.degrees(tilt), 5.71, places=1)
        self.assertEqual(tilt_pwm, 1324)
        self.assertEqual(pan, 0.0)
        recovered_pan, recovered_tilt = angles_from_pwm(pan_pwm, tilt_pwm)
        self.assertAlmostEqual(recovered_pan, pan, places=3)
        self.assertAlmostEqual(recovered_tilt, tilt, places=3)

    def test_public_pwm_mapping_uses_positive_up_tilt(self) -> None:
        self.assertEqual(public_angles_from_pwm(1000, 1000), (-180.0, -30.0))
        self.assertEqual(public_angles_from_pwm(1500, 1400), (0.0, 0.0))
        self.assertEqual(public_angles_from_pwm(2000, 2000), (180.0, 45.0))

    def test_scene_parser_reads_physical_tower_joints(self) -> None:
        message = {
            "msg": {
                "model": [
                    {
                        "name": "tower-1",
                        "joint": [
                            {"name": "tower-1::pan_joint", "angle": [math.pi / 5]},
                            {"name": "tower-1::tilt_joint", "angle": [-math.pi / 12]},
                        ],
                    }
                ]
            }
        }

        pan, tilt_up = tower_joint_angles_from_scene(message, "tower-1")

        self.assertAlmostEqual(pan, 36.0)
        self.assertAlmostEqual(tilt_up, 15.0)

    def test_scene_parser_reads_model_pose_and_yaw(self) -> None:
        message = {
            "topic": "~/scene",
            "msg": {
                "model": [
                    {
                        "name": "target_vessel",
                        "id": 9,
                        "pose": {
                            "position": {"x": 12, "y": 34, "z": 0},
                            "orientation": yaw_quaternion(math.pi / 2),
                        },
                    }
                ]
            },
        }

        model = models_from_scene(message)["target_vessel"]

        self.assertEqual(model.model_id, 9)
        self.assertEqual((model.x, model.y), (12.0, 34.0))
        self.assertAlmostEqual(model.yaw_rad, math.pi / 2)

    def test_pose_topic_parser_reads_direct_pose_fields(self) -> None:
        model = models_from_scene(
            {
                "msg": {
                    "model": [
                        {
                            "name": "training_vessel",
                            "id": 42,
                            "position": {"x": 12, "y": 34, "z": 0},
                            "orientation": yaw_quaternion(math.pi),
                        }
                    ]
                }
            }
        )["training_vessel"]

        self.assertEqual((model.x, model.y), (12.0, 34.0))
        self.assertAlmostEqual(abs(model.yaw_rad), math.pi)

    def test_projected_prelabel_is_valid_yolo_box(self) -> None:
        tower = ModelPose("tower-1", 1, 0.0, 0.0, 100.0, 0.0)
        target = PoseSpec("boat", 1000.0, 0.0, 90.0)
        _, _, pan, tilt = aim_pwm(tower, target)

        box = projected_boat_box(
            width=640,
            height=360,
            hfov_deg=60.0,
            vfov_deg=36.1,
            tower=tower,
            target=target,
            camera_pan_rad=pan,
            camera_tilt_rad=tilt,
        )

        self.assertIsNotNone(box)
        values = [float(value) for value in yolo_line(box, 640, 360).split()]
        self.assertEqual(values[0], 0.0)
        self.assertTrue(all(0.0 <= value <= 1.0 for value in values[1:]))

    def test_factory_quaternion_is_json_serializable(self) -> None:
        self.assertIn('"w"', json.dumps(yaw_quaternion(math.pi)))

    def test_runtime_replacement_orders_delete_before_factory(self) -> None:
        bridge = object.__new__(GazeboBridge)
        messages: list[tuple[str, dict[str, object]]] = []
        bridge.publish = lambda topic, message: messages.append((topic, message))
        observed = ModelPose("training_vessel", 42, 12.0, 34.0, 0.0, math.pi / 2)
        waits: list[dict[str, object]] = []

        def wait_for_pose_update(_name, **kwargs):
            waits.append(kwargs)
            return observed

        bridge.wait_for_pose_update = wait_for_pose_update

        with patch("scripts.collect_horizontal.time.sleep"):
            spawned = bridge.replace_vessel(
                PoseSpec("one", 12.0, 34.0, 90.0),
                delete_name="target_vessel",
                delete_existing=True,
                model_name="training_vessel",
                model_type="fishing_vessel",
            )

        self.assertEqual([topic for topic, _ in messages], ["~/entity_delete", "~/factory"])
        self.assertEqual(messages[1][1]["position"], {"x": 12.0, "y": 34.0, "z": 0.0})
        self.assertEqual((spawned.x, spawned.y), (12.0, 34.0))
        self.assertEqual(waits[0]["pose"], PoseSpec("one", 12.0, 34.0, 90.0))
        self.assertEqual(waits[0]["topics"], ("~/model/info", "~/pose/info"))

    def test_subsequent_pose_uses_model_modify(self) -> None:
        bridge = object.__new__(GazeboBridge)
        messages: list[tuple[str, dict[str, object]]] = []
        bridge.publish = lambda topic, message: messages.append((topic, message))
        original = ModelPose("target_vessel", 42, 1.0, 2.0, 0.0, 0.0)
        expected = PoseSpec("two", 12.0, 34.0, 180.0)
        moved = ModelPose("target_vessel", 42, 12.0, 34.0, 0.0, math.pi)
        bridge.wait_for_pose_update = lambda _name, **_kwargs: moved

        with patch("scripts.collect_horizontal.time.sleep"):
            observed = bridge.move_vessel(expected, model=original)

        self.assertEqual([topic for topic, _ in messages], ["~/model/modify"])
        self.assertEqual(messages[0][1]["id"], 42)
        self.assertEqual(messages[0][1]["position"], {"x": 12.0, "y": 34.0, "z": 0.0})
        self.assertEqual(observed, moved)

    def test_move_reconnects_and_resends_after_lost_ack(self) -> None:
        bridge = object.__new__(GazeboBridge)
        messages: list[tuple[str, dict[str, object]]] = []
        bridge.publish = lambda topic, message: messages.append((topic, message))
        reconnects: list[bool] = []
        bridge.reconnect = lambda: reconnects.append(True)
        moved = ModelPose("training_vessel", 42, 12.0, 34.0, 0.0, math.pi)
        results = iter((TimeoutError("lost ack"),))
        bridge.scene = lambda: {"training_vessel": moved}

        def wait_for_pose_update(_name, **_kwargs):
            result = next(results)
            if isinstance(result, Exception):
                raise result
            return result

        bridge.wait_for_pose_update = wait_for_pose_update
        with patch("scripts.collect_horizontal.time.sleep"):
            observed = bridge.move_vessel(
                PoseSpec("two", 12.0, 34.0, 180.0),
                model=ModelPose("training_vessel", 42, 1.0, 2.0, 0.0, 0.0),
            )

        self.assertEqual(observed, moved)
        self.assertEqual(len(messages), 1)
        self.assertEqual(reconnects, [True])

    def test_camera_rejects_repeated_cached_jpeg(self) -> None:
        camera = object.__new__(LatestFrameCamera)
        camera.name = "tower-1"
        camera._condition = threading.Condition()
        camera._error = None
        camera._sequence = 2
        camera._latest = CameraFrame(
            "tower-1",
            b"same-jpeg",
            datetime.now(timezone.utc),
        )

        with self.assertRaisesRegex(TimeoutError, "repeating the previous JPEG"):
            camera.wait_after(1, timeout_s=0.001, different_from=b"same-jpeg")

        camera._latest = CameraFrame(
            "tower-1",
            b"new-jpeg",
            datetime.now(timezone.utc),
        )
        sequence, frame = camera.wait_after(
            1,
            timeout_s=0.001,
            different_from=b"same-jpeg",
        )
        self.assertEqual(sequence, 2)
        self.assertEqual(frame.jpeg, b"new-jpeg")

    def test_tower_start_retries_missing_heartbeat(self) -> None:
        class FakeController:
            def __init__(self, online: bool) -> None:
                self.online = online
                self.closed = False

            def start(self):
                if not self.online:
                    raise MavProxyConnectionError("no heartbeat")
                return self

            def close(self) -> None:
                self.closed = True

        controllers = [FakeController(False), FakeController(True)]

        class FakeTower:
            name = "tower-1"

            def controller(self, **_kwargs):
                return controllers.pop(0)

        with patch("scripts.collect_horizontal.time.sleep"):
            observed = start_tower_controller(FakeTower(), attempts=2)

        self.assertTrue(observed.online)
        self.assertFalse(controllers)


if __name__ == "__main__":
    unittest.main()
