# Dominion Dynamics WHITEOUT controller scaffold

WHITEOUT is a small, safety-first Python 3.11+ foundation for ingesting simulator imagery and telemetry, estimating a target position, maintaining a track, and recommending observation actions. It does not control vehicles.

## Safety defaults

- No code in this repository arms, moves, resets, rebuilds, or changes simulator settings.
- MAVLink uses `udpout` because each simulator MAVProxy endpoint is an `udpin` listener. The adapter exposes telemetry polling and no flight-control interface. An explicit `connect(initiate_telemetry=True)` may send one benign GCS heartbeat to establish telemetry; the default sends nothing.
- Coordinator outputs are plain `ActionRecommendation` values. They are not executed.
- There is no local HTTP server. `TrackApiClient` only makes an outbound POST to the configured Dominion endpoint.
- Outbound track submission requires both `track_api.allow_submission: true` in configuration and explicit confirmation from the caller. The test submission utility requires `--confirm`.
- Network utilities do nothing unless their explicit confirmation flag is present.

## Setup

Create a Python 3.11 or newer virtual environment, then install the package in editable mode. The unit tests need no third-party packages. Install only the optional features you use: `whiteout[yaml]` for YAML configuration, `whiteout[camera]` for OpenCV decoding, or `whiteout[mavlink]` for telemetry.

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
- `geolocation.py`: pinhole ray intersection with a locally flat water plane and propagated uncertainty.
- `tracker.py`: alpha-beta constant-velocity track filtering.
- `coordinator.py`: SEARCH, CONFIRM, TRACK, and REACQUIRE recommendation logic.
- `track_api.py`: safety-gated outbound Dominion client and internal-track payload mapping.
- `telemetry_log.py`: append-only JSONL event records.
- `main.py`: bounded camera-to-detector observation pipeline and safety-gated CLI.

The flat-water estimator is intentionally local and approximate. Camera calibration, mounting angles, water elevation, and uncertainty values must be measured for a real deployment.

## Detector integration

The detector implementation is the teammate integration point. Implement the `Detector` protocol with a class whose `detect(frame)` method returns typed `Detection` values. Keep model loading in the implementation constructor so importing WHITEOUT remains usable offline. Convert detections to `GeoEstimate` values using the camera calibration and the matching platform pose before updating the tracker. The included `NoOpDetector` is the default safe placeholder. The current coordinator produces recommendations only; it does not execute or transmit vehicle actions.

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

No step should add flight-command transmission to this project. A separate, reviewed control system may consume recommendations if the deployment requires it.

The built-in placeholder pipeline can process a bounded camera sample with `whiteout --config config.yaml --observe-camera NAME --max-frames 10 --confirm-network`. It uses `NoOpDetector`, makes recommendations only, and never submits tracks.
