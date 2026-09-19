# Dominion Dynamics WHITEOUT controller scaffold

WHITEOUT is a small, safety-first Python 3.11+ foundation for ingesting simulator imagery and telemetry, estimating a target position, maintaining a track, recommending observation actions, and explicitly controlling simulator assets through MAVProxy.

## Safety defaults

- Nothing arms or moves automatically. Vehicle commands are transmitted only when a caller explicitly opens a `MavProxySession` and sends them.
- MAVLink uses `udpout` because each simulator MAVProxy endpoint is an `udpin` listener. The adapter exposes telemetry polling and no flight-control interface. An explicit `connect(initiate_telemetry=True)` may send one benign GCS heartbeat to establish telemetry; the default sends nothing.
- Simulator control launches `mavproxy.py` with `--master=udpout:<host>:<port>` and writes validated console commands to that process. It does not implement a parallel direct-pymavlink control path.
- Coordinator outputs are plain `ActionRecommendation` values. They are not executed.
- There is no local HTTP server. `TrackApiClient` only makes an outbound POST to the configured Dominion endpoint.
- Outbound track submission requires both `track_api.allow_submission: true` in configuration and explicit confirmation from the caller. The test submission utility requires `--confirm`.
- Network utilities do nothing unless their explicit confirmation flag is present.

## Setup

Create a Python 3.11 or newer virtual environment, then install the package in editable mode. The unit tests need no third-party packages. Install only the optional features you use: `whiteout[yaml]` for YAML configuration, `whiteout[camera]` for OpenCV decoding, `whiteout[mavlink]` for direct telemetry polling, or `whiteout[mavproxy]` for simulator control.

Copy `.env.example` to `.env` only if your process manager loads it. The package does not silently load dotenv files. Copy `config.example.yaml` to `config.yaml`, set `placement_owner`, and validate it with `whiteout --config config.yaml --check-config`. JSON configuration works without PyYAML.

Run the offline suite with:

```text
PYTHONPATH=src python -m unittest discover -s tests -v
```

## Architecture

- `models.py`: immutable typed observations, estimates, tracks, and recommendations.
- `config.py`: validated configuration and `${NAME}` or `${NAME:-fallback}` substitution.
- `camera.py`: reconnecting MJPEG byte ingestion with optional OpenCV decoding.
- `detector.py`: detector protocol plus a safe detector that returns no observations.
- `mavlink.py`: optional telemetry-only `GLOBAL_POSITION_INT` polling over `udpout`, plus the explicit discovery heartbeat.
- `mavproxy.py`: managed MAVProxy subprocess sessions used for explicit simulator control.
- `objects.py`: typed copter, plane, reserved boat, and tower descriptions with validated MAVProxy console commands.
- `geolocation.py`: pinhole ray intersection with a locally flat water plane and propagated uncertainty.
- `tracker.py`: alpha-beta constant-velocity track filtering.
- `coordinator.py`: SEARCH, CONFIRM, TRACK, and REACQUIRE recommendation logic.
- `track_api.py`: safety-gated outbound Dominion client and internal-track payload mapping.
- `telemetry_log.py`: append-only JSONL event records.
- `main.py`: bounded camera-to-detector observation pipeline and safety-gated CLI.

The flat-water estimator is intentionally local and approximate. Camera calibration, mounting angles, water elevation, and uncertainty values must be measured for a real deployment.

## Repository object classes

`whiteout.objects` contains `Copter`, `Plane`, `Boat`, and `Tower` classes. Each class owns its arctic-sim MAVProxy endpoint, supported modes, and relevant command builders. The rover is intentionally absent because it is not part of the WHITEOUT fleet. Install MAVProxy before opening a session:

```text
pip install -e ".[mavproxy]"
```

```python
from whiteout.objects import Copter, Tower

copter = Copter(host="sim.example")
print(copter.mavproxy_start_command())  # mavproxy.py --master=udpout:sim.example:14550
print(copter.mode("GUIDED"))            # mode GUIDED
print(copter.takeoff(20))               # takeoff 20

tower_command = Tower.two().pan(1200)
```

To transmit commands, explicitly open a MAVProxy session. The subprocess uses an argument vector rather than a shell and connects in `udpout` mode, as required by arctic-sim's `udpin` endpoints:

```python
copter = Copter(host="localhost")
with copter.mavproxy_session() as mavproxy:
    mavproxy.send(copter.mode("GUIDED"))
    mavproxy.send(copter.arm())
    mavproxy.send(copter.takeoff(20))
```

The `Boat` class records the reserved ArduRover-based boat role, but its `bundled` flag is false because arctic-sim does not currently include that model. A boat session therefore cannot be opened. Towers do not expose arming, and their sessions automatically load MAVProxy's optional `servo` module for `pan()` and `tilt()`. Unsupported modes and out-of-range PWM values raise an error before a command is transmitted.

## Detector integration

The detector implementation is the teammate integration point. Implement the `Detector` protocol with a class whose `detect(frame)` method returns typed `Detection` values. Keep model loading in the implementation constructor so importing WHITEOUT remains usable offline. Convert detections to `GeoEstimate` values using the camera calibration and the matching platform pose before updating the tracker. The included `NoOpDetector` is the default safe placeholder. The current coordinator produces recommendations only; it is not wired to a MAVProxy session and does not execute or transmit vehicle actions.

## Placement owner and simulator location

Set `placement_owner` to the person or service responsible for camera placement and calibration. Treat `unassigned` as a deployment blocker. That owner should maintain the mapping between each configured camera, its platform, and calibrated pose. Resolution and FOV values are included in the example configuration, but they are not enough for geolocation. Camera mount orientation and full extrinsics must be calibrated, especially for both towers and both aircraft, whose attitude and mounting offsets directly affect ground intersection.

For a simulator on the same machine, set `SIM_HOST=127.0.0.1`. For a remote simulator, use its resolvable hostname or private network address. Camera URLs and MAVLink listeners are assembled from `SIM_HOST` plus the configured ports. Never put credentials in configuration. Firewall and routing setup remain outside this controller.

The official role mapping is:

| Role | Camera | MAVLink |
| --- | ---: | ---: |
| `quadcopter` | 8600 | 14550 |
| `fixed-wing` | 8610 | 14560 |
| `tower-1` | 8630 | 14580 |
| `tower-2` | 8640 | 14590 |

Every camera uses `/stream`. MAVLink addresses have the form `udpout:<SIM_HOST>:<port>`.

## Dominion track submission

Set `track_api.endpoint` to the approved Dominion endpoint and keep a persistent `track_api.name`, such as `Sierra One`. The client maps an internal track to `name`, `lat`, and `lon`, adding heading in degrees clockwise from north when velocity is available. `include_speed` defaults to false because the official speed units are not confirmed. Enable it only after confirming the endpoint's expected units; the current internal magnitude is metres per second.

Submission remains disabled by default. A POST occurs only when configuration sets `allow_submission: true` and the individual caller passes explicit confirmation. Tests inject a fake opener and make no network requests.

## One-day workflow

1. Assign the placement owner and record camera intrinsics, mounting pose, and water-plane assumptions.
2. Validate configuration offline and run the unit tests.
3. With authorization, use `scripts/check_connections.py --confirm-network` to inspect camera TCP reachability. It does not probe UDP endpoints.
4. Record a short bounded sample using `scripts/record_cameras.py --camera NAME --frames COUNT --confirm-network`.
5. Implement and evaluate a detector against saved frames, without involving the simulator.
6. Replay detections through geolocation, tracking, and coordination while inspecting JSONL logs.
7. Review uncertainties and recommendation transitions with the placement owner.
8. If Dominion submission is approved, set both the endpoint and config opt-in. Exercise `scripts/submit_test_track.py --confirm` only against that approved endpoint.

Keep recommendation generation separate from command transmission. A caller must explicitly decide which reviewed recommendations, if any, to send through a `MavProxySession`.

The built-in placeholder pipeline can process a bounded camera sample with `whiteout --config config.yaml --observe-camera NAME --max-frames 10 --confirm-network`. It uses `NoOpDetector`, makes recommendations only, and never submits tracks.
