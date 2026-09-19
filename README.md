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

The normal public API is a set of typed Python controllers. They start MAVProxy, wait for the vehicle heartbeat, validate arguments, and transmit internally; application code does not need to build console strings:

```python
from whiteout import Copter, CopterMode, Plane, PlaneMode, Tower

with Copter(host="10.99.0.1").controller() as copter:
    copter.set_mode(CopterMode.GUIDED)
    copter.arm()
    copter.takeoff(10)
    copter.autoland()

with Plane(host="10.99.0.1").controller() as plane:
    plane.set_mode(PlaneMode.LOITER)

with Tower.one("10.99.0.1").controller() as tower:
    tower.pan(1200)
    tower.tilt(1700)
```

`CopterMode` and `PlaneMode` are separate enums, so plane-only and copter-only modes cannot be mixed accidentally. Towers do not expose arming. Their pan and tilt methods use MAVProxy's built-in `cmdlong` command with `MAV_CMD_DO_SET_SERVO`; no unavailable third-party servo module is required. Unsupported modes and out-of-range PWM values raise an error before transmission. Low-level `MavProxyCommand` builders remain available for unusual commands, but are not needed for normal control.

For a bounded Python test flight, install `whiteout[mavproxy]`, start arctic-sim, and run one of these. The required flag is deliberate because these commands arm and move the simulated aircraft:

```text
python scripts/test_fly.py quadcopter --confirm-flight
python scripts/test_fly.py fixed-wing --confirm-flight
```

The quadcopter enters GUIDED mode, takes off to 10 m, holds for 15 seconds, then enters LAND mode. The fixed-wing enters TAKEOFF mode, climbs for 45 seconds, applies the simulator's gentle belly-landing flare profile, uploads `missions/arctic_sim_fixed_wing_land.waypoints`, waits for MAVProxy to confirm the upload, and enters AUTO. Both flows poll MAVProxy's heartbeat and exit as soon as automatic disarm is confirmed; zero throttle alone is not treated as completion. The fixed-wing then returns to MANUAL and clears the landing mission, which prevents ArduPlane's `In landing sequence` pre-arm rejection on the next flight. Use `--altitude-m`, `--hold-seconds`, and `--landing-timeout-seconds` to change the bounded defaults.

The default fixed-wing path is deliberately hard-coded and repeatable. Its desired stopped position is the captured starting position `71.9982129, -94.8420161`. Because the prior flight stopped about 52 m before its commanded `NAV_LAND` coordinate, the mission aims 52 m beyond that position at `71.9981315, -94.8435044`; this makes the expected rollout end at the starting position instead of treating it as the touchdown aim. The final course is `259.96°`, five degrees clockwise from the captured heading. The first navigated landing waypoint is the supplied ocean-side point `71.9992990, -94.8431670` at 76 m, above the requested 75.99 m clearance. It remains north of the mountain at `71.995786, -94.838870`, makes its large alignment turn at 55 m, and follows a straight descent through 35 m and 12 m gates. Supply `--landing-mission PATH` only to intentionally override this route.

The `Boat` class records the reserved boat role, but its `bundled` flag is false because arctic-sim does not currently include that model. A boat session therefore cannot be opened.

`whiteout.ARCTIC_SIM_FLEET` provides the live deployment endpoints at `10.99.0.1` in this fixed order: quadcopter, fixed-wing, tower-1, tower-2. Its MAVProxy commands use ports 14550, 14560, 14580, and 14590; the paired camera streams use ports 8600, 8610, 8630, and 8640 with path `/stream`.

The ordinary unit suite verifies the exact commands, URLs, and ordering without network access. To additionally smoke-test the four live camera streams and launch MAVProxy against all four assets, run:

```text
WHITEOUT_LIVE_ARCTIC_SIM=1 PYTHONPATH=src python -m unittest tests.test_arctic_sim -v
```

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
