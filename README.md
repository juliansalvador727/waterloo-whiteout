# Dominion Dynamics WHITEOUT controller

WHITEOUT is a safety-focused Python 3.11+ controller for ArcticSim. It ingests camera imagery and MAVLink telemetry, provides target geolocation and tracking components, recommends observation actions, and exposes explicit MAVProxy-backed controls for the simulated fleet.

> **This repository does not place towers or other simulator assets.** ArcticSim owns all asset placement through its own `arctic-sim/.env` `ASSET_N` entries. `waterloo-whiteout` is a separate controller repository. Do not copy ArcticSim into this repository or add its placement entries here.

## Safety boundaries

- Nothing arms or moves automatically. Vehicle commands are transmitted only when a caller explicitly opens a controller or `MavProxySession` and invokes a command.
- `ReadOnlyMavlink` is a separate telemetry-only adapter. It connects with `udpout`, sends nothing by default, and has no flight-control methods. `connect(initiate_telemetry=True)` may send one benign GCS heartbeat to establish telemetry.
- Simulator control launches `mavproxy.py` with `--master=udpout:<host>:<port>` and sends validated console commands to that process. There is no parallel direct-pymavlink control path.
- `scripts/test_fly.py` requires `--confirm-flight` because it arms and moves an aircraft.
- Coordinator outputs are `ActionRecommendation` data. They are not wired to the control layer and are never executed automatically.
- There is no local HTTP server. `TrackApiClient` only makes an outbound POST to the configured Dominion endpoint.
- Track submission requires both `track_api.allow_submission: true` and explicit confirmation from the caller. Network utilities also require their documented confirmation flags.

## Quick start

### 1. Install the controller

Create and activate a Python 3.11 or newer virtual environment:

```sh
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[yaml,camera,mavlink]'
```

On Windows PowerShell, activate with `.venv\Scripts\Activate.ps1`. The optional extras are:

- `yaml`: PyYAML configuration support
- `camera`: OpenCV frame decoding
- `mavlink`: direct, telemetry-only pymavlink polling
- `mavproxy`: MAVProxy-backed simulator control
- `dashboard`: browser dashboard, recording/replay, and Arctic map projection

The offline unit tests need no third-party packages. Install the control extra before opening a simulator controller:

```sh
python -m pip install -e '.[mavproxy]'
```

### 2. Configure and verify offline

```sh
cp config.example.yaml config.yaml
export SIM_HOST=127.0.0.1
```

Edit `config.yaml` and replace `placement_owner: unassigned` with the teammate responsible for simulator placement and camera calibration. Treat an unassigned owner as a deployment blocker. When ArcticSim runs on another machine, use its resolvable hostname or private address instead of `127.0.0.1`.

Validate configuration and run the offline suite:

```sh
whiteout --config config.yaml --check-config
python -m unittest discover -s tests -v
```

The package does not automatically load dotenv files. Do not commit `.env`, `config.yaml`, credentials, recordings, or telemetry logs.

### 3. Check an authorized live camera

Coordinate with the teammate hosting ArcticSim first. These bounded commands make network connections but send no flight commands or track submissions:

```sh
python scripts/check_connections.py --config config.yaml --confirm-network
whiteout --config config.yaml --observe-camera tower-1 --max-frames 10 --confirm-network
```

The built-in observation command uses `NoOpDetector`, so zero detections are expected. The TCP checker tests camera ports only and does not probe MAVLink UDP endpoints.

## Implemented components

- validated YAML or JSON configuration with `${NAME}` and `${NAME:-fallback}` substitution
- reconnecting MJPEG ingestion with optional OpenCV decoding
- a typed detector interface and safe `NoOpDetector`
- telemetry-only `GLOBAL_POSITION_INT` polling through `ReadOnlyMavlink`
- typed copter, plane, tower, and reserved boat descriptions
- managed MAVProxy sessions and typed controllers for explicit simulator control
- validated QGC WPL search mission upload for both aircraft, separate explicit start, pause/resume, RTL/abort, and safe clearing
- a separate runway-specific fixed-wing landing mission
- local flat-water pixel geolocation with uncertainty
- an alpha-beta constant-velocity tracker
- SEARCH, CONFIRM, TRACK, REACQUIRE, RETURN, and ABORT recommendations with offline manual or synthetic detection injection
- append-only JSONL logging and a safety-gated Dominion track client

The observation pipeline is still a scaffold. It has no production detector and does not connect telemetry, geolocation, tracking, control, or submission end to end. `ReadOnlyMavlink` reads position plus synchronized `ATTITUDE` samples without blocking. Missing, stale, or unsynchronized attitude remains explicitly absent rather than appearing as live zero values. Do not trust aircraft geolocation until calibrated camera extrinsics are wired and each frame's observation metadata reports synchronized pose and attitude.

## ArcticSim endpoints

The configured role mapping is:

| Role | Camera URL | MAVLink address |
| --- | --- | --- |
| `quadcopter` | `http://<SIM_HOST>:8600/stream` | `udpout:<SIM_HOST>:14550` |
| `fixed-wing` | `http://<SIM_HOST>:8610/stream` | `udpout:<SIM_HOST>:14560` |
| `tower-1` | `http://<SIM_HOST>:8630/stream` | `udpout:<SIM_HOST>:14580` |
| `tower-2` | `http://<SIM_HOST>:8640/stream` | `udpout:<SIM_HOST>:14590` |

All cameras use `/stream`. ArcticSim's MAVProxy endpoints are `udpin` listeners, so both the telemetry adapter and managed MAVProxy sessions connect with `udpout`.

`whiteout.ARCTIC_SIM_FLEET` exposes these four deployed assets in the same order, using the default live host `10.99.0.1`. The `Boat` class reserves port 14570, but its `bundled` flag is false because ArcticSim does not currently include that model. A boat session cannot be opened.

## MAVProxy hardware access

`Copter`, `Plane`, and `Tower` create typed controllers that start MAVProxy, wait for a vehicle heartbeat, validate values, and transmit commands. Application code does not need to construct console strings:

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

`CopterMode` and `PlaneMode` are separate enums, so modes cannot be mixed between vehicle types. Towers do not support arming. Pan and tilt use MAVProxy's built-in `cmdlong` command with `MAV_CMD_DO_SET_SERVO`, without an optional servo module. Unsupported operations, invalid modes, and PWM values outside 1000 to 2000 are rejected before transmission.

Low-level `MavProxyCommand` builders remain available for unusual cases. Building a command does not execute it. Transmission only occurs through a running `MavProxySession` or typed controller.

To smoke-test the four live camera streams and connect a controller to each asset, opt in explicitly:

```sh
WHITEOUT_LIVE_ARCTIC_SIM=1 PYTHONPATH=src python -m unittest tests.test_arctic_sim -v
```

This live test launches MAVProxy against real simulator endpoints. The normal unit suite uses fakes and makes no network connections.

## Search mission workflow

The canonical routes are `missions/fixed_wing_search.waypoints` and `missions/quadcopter_search.waypoints`. They are specific to the active ArcticSim Fort Ross build in `out/fort_ross`: its generated 6,132.3 m target course has seed `781223750` and was segment-verified land-free with 336.4 m minimum clearance. These routes are deterministic coverage patterns derived from that corridor. No route optimization was performed.

The fixed-wing route flies at 200 m relative altitude. It sweeps both sides of the full generated boat corridor with approximately 100 m offsets, a 200 m total swath, and 100 m rounded end turns. Its closed search loop plus the initial transit is approximately 13.9 km. This full-corridor coverage matters because the active simulator uses `SHIP_START=random`, so the vessel's phase along the course is not known. The quadcopter first climbs at its documented home, `71.995807, -94.839300`, then transits at 90 m relative altitude to a local open-water sector. Its closed coverage loop is approximately 800 m long by 100 m wide, with an approximately 2.5 km total mission path including transit. It does not claim to cover the full 6.5 km map and does not assume that the target has been detected.

Regenerate and revalidate both files whenever `COURSE_SEED`, the active site, or generated terrain changes. Regenerate the affected aircraft file if its spawn changes. Validation must use the new `terrain.json` course plus the generated DEM or heightmap, and must check every straight mission segment rather than waypoint positions alone. Also review terrain clearance, aircraft turn performance, camera footprint, battery endurance, geofence, and current simulator state before live use.

In application code, open the appropriate typed controller and follow this deliberate sequence:

1. Call `upload_search_mission(path)`. It validates QGC WPL structure and search-only commands, uploads through MAVProxy, and waits for MAVProxy's `Sent all` confirmation. It does not arm or enter `AUTO`.
2. Arm and establish a safe takeoff or launch using the vehicle-specific procedure.
3. Call `start_search()` explicitly to enter `AUTO` only after operator review.
4. Use `pause_search()` to enter vehicle `LOITER`, `resume_search()` to re-enter `AUTO`, or `abort_search()` to enter `RTL`.
5. Disarm before `clear_search_mission()`. Clearing switches to `STABILIZE` for the copter or `MANUAL` for the plane and refuses to run while armed.

`SearchOrchestrator` consumes a `Detection` or `None` and emits inert `SearchRecommendation` data. Manual and synthetic detections can be injected offline without a detector. A first candidate remains in `CONFIRM` and never produces a pursuit recommendation; only repeated confirmation reaches `TRACK`. RETURN and ABORT recommend RTL, but nothing in orchestration transmits to MAVProxy.

## Bounded simulator flights and waypoint landing

Start ArcticSim, install `whiteout[mavproxy]`, and run one of the following only when an authorized simulator operator expects the aircraft to move:

```sh
python scripts/test_fly.py quadcopter --confirm-flight
python scripts/test_fly.py fixed-wing --confirm-flight
```

The quadcopter enters GUIDED mode, takes off to 10 m, holds for 15 seconds, and enters LAND mode. The fixed-wing enters TAKEOFF mode, climbs for 45 seconds, applies a conservative belly-landing flare configuration, uploads `missions/arctic_sim_fixed_wing_land.waypoints`, waits for MAVProxy to confirm the upload, and enters AUTO.

Both flows poll the MAVProxy heartbeat and finish only after automatic disarm is confirmed. Zero throttle alone is not treated as completion. After the fixed-wing lands, the script returns it to MANUAL and clears the mission so ArduPlane does not reject the next flight with `In landing sequence`.

The default fixed-wing path is deliberately hard-coded and repeatable. Its desired stopped position is the captured starting position `71.9982129, -94.8420161`. Because the prior flight stopped about 52 m before its commanded `NAV_LAND` coordinate, the mission aims 52 m beyond that position at `71.9981315, -94.8435044`; this makes the expected rollout end at the starting position instead of treating it as the touchdown aim. The final course is `259.96°`, five degrees clockwise from the captured heading. The first navigated landing waypoint is the supplied ocean-side point `71.9992990, -94.8431670` at 76 m, above the requested 75.99 m clearance. It remains north of the mountain at `71.995786, -94.838870`, makes its large alignment turn at 55 m, and follows a straight descent through 35 m and 12 m gates. Supply `--landing-mission PATH` only to intentionally override this route.

Use `--altitude-m`, `--hold-seconds`, and `--landing-timeout-seconds` to change the bounded defaults. Use `--host` when the simulator is not at `10.99.0.1`.

> **Runway-specific mission warning:** The bundled fixed-wing mission is tied to the default ArcticSim runway declared by `ASSET_3` in ArcticSim's `.env.example`. Its final approach is rotated 10 degrees clockwise from the original runway vector when viewed from above. If the aircraft spawn or runway changes, provide a matching QGC WPL file with `--landing-mission PATH`. Never reuse the bundled coordinates at another site.

The landing mission is not a search route. Search upload rejects its landing commands, and landing remains available only through the plane-specific `autoland()` flow.

## Tower placement and camera calibration

Tower placement belongs entirely to ArcticSim. Only the teammate hosting the simulator should edit `arctic-sim/.env`, and the team should coordinate before changing a shared scene. This controller can pan and tilt an existing tower, but it cannot place, relocate, rebuild, restart, or reset simulator assets.

An ArcticSim placement entry has this documented shape:

```dotenv
ASSET_N=tower,<asset-name>,<latitude>,<longitude>
```

The documented default tower entries are:

```dotenv
ASSET_2=tower,tower-1,71.980671,-94.853711
ASSET_5=tower,tower-2,72.011778,-94.804721
```

ArcticSim derives tower elevation from terrain. Camera pan and tilt are set after startup through the typed tower controller, not in the placement entry. After an authorized `.env` edit, the ArcticSim host must follow ArcticSim's documented save, rebuild, and restart procedure. Those are simulator operations, not controller commands.

After the host reports the simulator ready, verify both views:

- tower 1: `http://<SIM_HOST>:8630/stream`
- tower 2: `http://<SIM_HOST>:8640/stream`

Bounded recordings require explicit network confirmation:

```sh
python scripts/record_cameras.py --config config.yaml --camera tower-1 --frames 30 --confirm-network
python scripts/record_cameras.py --config config.yaml --camera tower-2 --frames 30 --confirm-network
```

Check line of sight, target scale, overlapping water coverage, and camera orientation. A correctly placed tower can still show only sky or terrain if pan or tilt is unsuitable. Record the intrinsics, mount pose, water-plane assumptions, and uncertainty for every camera. Resolution and field of view alone are not enough for geolocation.

## Detector and tracking integration

Implement the `Detector` protocol from `src/whiteout/detector.py` with `detect(frame: CameraFrame) -> list[Detection]`. Each detection needs the source camera name, target pixel, confidence, and preferably the frame timestamp. Keep model loading in the implementation constructor so imports and offline tests do not require model weights or network access.

The remaining integration sequence is:

1. Select the detector instead of `NoOpDetector`.
2. Associate each detection with synchronized platform or tower pose.
3. Convert the target pixel to a `GeoEstimate` using calibrated intrinsics and pose.
4. Update `ConstantVelocityTracker`.
5. Pass the estimate to `Coordinator` and log the result.
6. Submit the resulting `Track` only after both submission gates are enabled.

Keep recommendation generation separate from command transmission. A caller must explicitly decide which reviewed recommendations, if any, to send through a controller.

## Dominion submission API

Set the approved endpoint through the environment and keep a persistent reporting name:

```sh
export DOMINION_TRACK_API_ENDPOINT='https://approved-endpoint.example'
export DOMINION_TRACK_NAME='Sierra One'
```

Then set `track_api.allow_submission: true` in local `config.yaml`. The test utility sends one synthetic track only when both the configuration gate and command confirmation are present:

```sh
python scripts/submit_test_track.py --config config.yaml --confirm
```

Use that command only with an endpoint approved by Dominion. Payloads contain `name`, `lat`, and `lon`, plus heading when velocity is available. `include_speed` defaults to `false`; enable it only after Dominion confirms the expected units. The internal speed magnitude is metres per second.

## Troubleshooting

**`YAML configuration requires PyYAML`**

Activate the virtual environment and install the YAML extra:

```sh
python -m pip install -e '.[yaml]'
```

**A stream is unreachable**

Confirm `SIM_HOST`, the role's camera port, `/stream`, routing, and firewall rules with the simulator host. Confirm ArcticSim is already running. Do not start, rebuild, or reset the simulator from this repository.

**Detections are always empty**

`NoOpDetector` always returns an empty list. Wire a real detector, confirm it receives decodable frames, and validate its thresholds and labels against saved images.

**Geolocation is wrong**

Verify the camera-to-platform mapping, timestamps, focal lengths, optical center, mount rotation, live attitude, altitude reference, water elevation, and angle units. The estimator assumes a local flat water plane and is intentionally approximate.

**MAVProxy is unavailable or a controller never comes online**

Install `whiteout[mavproxy]`, confirm `mavproxy.py` is on the active environment's path, verify the matching UDP port, and confirm the simulator asset is already running. Controllers fail closed if the initial vehicle heartbeat is not observed.
The built-in placeholder pipeline can process a bounded camera sample with `whiteout --config config.yaml --observe-camera NAME --max-frames 10 --confirm-network`. It uses `NoOpDetector`, makes recommendations only, and never submits tracks.

## Unified mission dashboard

The dashboard targets one 1080p operator display. It combines all four camera feeds, connection and activity state, a tactical EPSG:3413 map, target confidence overlays, a best-target crop, observed-versus-predicted track state, and an event timeline. Its local HTML, CSS, and JavaScript have no external tile or CDN dependency.

Install and start a live read-only dashboard with explicit network confirmation:

```text
python -m venv .venv
source .venv/bin/activate
pip install -e '.[dashboard,yaml]'
cp config.example.yaml config.yaml
SIM_HOST=10.99.0.1 whiteout-dashboard --config config.yaml --mode live --confirm-network
```

Open `http://127.0.0.1:8070`. Add `--session recordings/run-001` to sample two frames per second per camera and record timestamped mission events. Replay the same session without a simulator connection:

```text
whiteout-dashboard --mode replay --session recordings/run-001
```

The standalone dashboard owns only camera ingestion. A central mission process can publish teammate outputs through `MissionStateStore`:

- `update_frame(...)` and `update_telemetry(...)` for Aaron's inputs.
- `update_detection(...)` for Jessi's typed detections and bounding boxes.
- `update_asset_activity(...)` for James's assignments, search paths, pan/tilt state, and valid camera footprints.
- `update_track(...)`, `update_recommendation(...)`, and `update_submission(...)` for integration state.

Search paths, FOV footprints, and accumulated coverage polygons use `(x, y)` pairs in EPSG:3413 metres. The dashboard will not draw a camera footprint unless the integration layer supplies one from calibrated pose and orientation data. Detection confidence heat is labelled as model analysis and is not presented as infrared imagery.
