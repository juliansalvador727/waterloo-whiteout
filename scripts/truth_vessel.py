#!/usr/bin/env python3
"""Read the simulator's ground-truth target-vessel position, for scoring our own.

There is no Dominion endpoint that hands out the boat's real position: :8010 is
the submission API (POST /api/tracks) and the simulator's own control API on
:8090 serves site metadata only. The truth the simulator itself uses is the
Gazebo pose of the `target_vessel` model, which gzweb already relays verbatim to
every browser over its WebSocket on :8080 as `~/pose/info`.

This script reads that stream, converts world metres to lat/lon the way
`terrain/make_world.py` does, and optionally diffs it against an estimate API
such as the WHITEOUT dashboard's `/api/state`. It is read-only: its WebSocket
messages only subscribe to simulator topics, and it never changes Gazebo state
or posts a track.

    python scripts/truth_vessel.py --host 10.99.0.1 --confirm-network
    python scripts/truth_vessel.py --host 10.99.0.1 --confirm-network \
        --compare-url http://127.0.0.1:8070/api/state \
        --jsonl recordings/truth.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
import urllib.request
from typing import Any, Iterator

# EPSG:3413 is polar stereographic true at 70 N, so a metre of grid is not a
# metre of ground anywhere else. build_terrain.py samples the DEM over
# `extent * k` grid metres to make the world span `extent` GROUND metres, and
# make_world.py therefore converts world -> projected as `centre + xy * k`.
# gzweb's own right-click readout omits k; at 72 N that is ~0.58% of the
# distance from the site centre, or 19 m at the edge of a 6.5 km site — the same
# order as the accuracy we are trying to measure, so we carry it here.
WGS84_E = 0.081819190842621


def point_scale_factor(lat_deg: float) -> float:
    """Grid metres per ground metre in EPSG:3413 at this latitude."""
    e = WGS84_E
    m = lambda p: math.cos(p) / math.sqrt(1 - e * e * math.sin(p) ** 2)  # noqa: E731
    t = lambda p: (  # noqa: E731
        math.tan(math.pi / 4 - p / 2)
        / ((1 - e * math.sin(p)) / (1 + e * math.sin(p))) ** (e / 2)
    )
    ts, la = math.radians(70.0), math.radians(lat_deg)
    return (m(ts) * t(la)) / (m(la) * t(ts))


def quaternion_yaw(q: dict[str, Any]) -> float:
    """Yaw in radians, CCW from world +X."""
    w, x, y, z = (float(q.get(k, 0.0)) for k in ("w", "x", "y", "z"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class WorldFrame:
    """World metres -> lat/lon, using the active site's EPSG:3413 bounds."""

    def __init__(self, site: dict[str, Any], *, true_scale: bool = True) -> None:
        from pyproj import Transformer  # imported late; dashboard extra

        bounds = site["bounds3413"]
        self.cx = (float(bounds["xmin"]) + float(bounds["xmax"])) / 2.0
        self.cy = (float(bounds["ymin"]) + float(bounds["ymax"])) / 2.0
        self.convergence_deg = float(site.get("convergence_deg") or 0.0)
        centre_lat = float(site["centre"]["lat"])
        self.k = point_scale_factor(centre_lat) if true_scale else 1.0
        self._inverse = Transformer.from_crs("EPSG:3413", "EPSG:4326", always_xy=True)

    def to_latlon(self, x: float, y: float) -> tuple[float, float]:
        lon, lat = self._inverse.transform(self.cx + x * self.k, self.cy + y * self.k)
        return lat, lon

    def true_bearing_deg(self, yaw_rad: float) -> float:
        """World yaw -> true compass bearing.

        World +Y bears `convergence_deg` from true north, so a world vector at
        yaw t has true bearing convergence + 90 - t.
        """
        return (self.convergence_deg + 90.0 - math.degrees(yaw_rad)) % 360.0


def fetch_site(host: str, timeout: float = 5.0) -> dict[str, Any]:
    url = f"http://{host}:8090/api/site"
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = json.loads(response.read())
    if not payload.get("ok"):
        raise RuntimeError(f"{url} reports no active site metadata")
    return payload


def fetch_json(url: str, timeout: float = 5.0) -> Any:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read())


def find_track(tracks: Any, name: str) -> dict[str, Any] | None:
    items = tracks
    if isinstance(tracks, dict):
        for key in ("tracks", "entities", "items", "data"):
            if isinstance(tracks.get(key), list):
                items = tracks[key]
                break
    if not isinstance(items, list):
        return None
    for item in items:
        if isinstance(item, dict) and item.get("name") == name:
            return item
    return None


def estimate_latlon(payload: Any, name: str | None = None) -> tuple[float, float] | None:
    """Extract an estimated lat/lon from dashboard state or a track collection."""
    candidate: Any = payload
    if isinstance(payload, dict) and isinstance(payload.get("target"), dict):
        candidate = payload["target"].get("track")
    elif name is not None:
        candidate = find_track(payload, name)
    elif isinstance(payload, dict):
        for key in ("tracks", "entities", "items", "data"):
            items = payload.get(key)
            if isinstance(items, list) and len(items) == 1:
                candidate = items[0]
                break
    elif isinstance(payload, list) and len(payload) == 1:
        candidate = payload[0]

    if not isinstance(candidate, dict):
        return None
    for lat_key, lon_key in (
        ("display_latitude", "display_longitude"),
        ("latitude", "longitude"),
        ("lat", "lon"),
    ):
        if candidate.get(lat_key) is not None and candidate.get(lon_key) is not None:
            return float(candidate[lat_key]), float(candidate[lon_key])
    return None


class EstimatePoller:
    """Fetch comparison estimates without blocking the pose WebSocket reader."""

    def __init__(
        self,
        url: str,
        *,
        name: str | None = None,
        interval: float = 0.5,
        timeout: float = 2.0,
    ) -> None:
        self.url = url
        self.name = name
        self.interval = interval
        self.timeout = timeout
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._result: tuple[tuple[float, float] | None, str | None, float | None] = (
            None,
            None,
            None,
        )
        self._thread = threading.Thread(
            target=self._run,
            name="truth-estimate-poller",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        # The worker is a daemon and may still be inside urlopen(). Do not let
        # an unavailable comparison endpoint delay the requested run deadline.
        self._thread.join(timeout=0.1)

    def snapshot(self) -> tuple[tuple[float, float] | None, str | None, float | None]:
        with self._lock:
            return self._result

    def _run(self) -> None:
        while not self._stop.is_set():
            fetched_at = time.time()
            try:
                coordinate = estimate_latlon(
                    fetch_json(self.url, timeout=self.timeout), self.name
                )
                error = None
            except (OSError, TimeoutError, TypeError, ValueError, json.JSONDecodeError) as exc:
                coordinate = None
                error = f"{type(exc).__name__}: {exc}"
            with self._lock:
                self._result = coordinate, error, fetched_at
            self._stop.wait(self.interval)


def _stamp_seconds(stamp: Any) -> float | None:
    if not isinstance(stamp, dict):
        return None
    try:
        return float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) / 1e9
    except (TypeError, ValueError):
        return None


class SimulationClock:
    """Estimate current simulation time using Gazebo's measured real-time factor."""

    def __init__(self) -> None:
        self.sim_time: float | None = None
        self.real_time: float | None = None
        self.rate: float | None = None
        self.received_at = 0.0
        self.paused = False

    def update(self, body: dict[str, Any], received_at: float) -> None:
        sim_time = _stamp_seconds(body.get("sim_time"))
        real_time = _stamp_seconds(body.get("real_time"))
        if (
            sim_time is not None
            and real_time is not None
            and self.sim_time is not None
            and self.real_time is not None
        ):
            real_delta = real_time - self.real_time
            sim_delta = sim_time - self.sim_time
            if real_delta > 0 and sim_delta >= 0:
                rate = sim_delta / real_delta
                if math.isfinite(rate):
                    self.rate = rate
        self.sim_time = sim_time
        self.real_time = real_time
        self.received_at = received_at
        self.paused = bool(body.get("paused"))

    def current(self, now: float) -> float | None:
        age = now - self.received_at
        if (
            self.sim_time is None
            or self.rate is None
            or self.paused
            or not 0 <= age <= 1.5
        ):
            return None
        return self.sim_time + age * self.rate


def _pose_from_message(message: dict[str, Any], model: str) -> dict[str, Any] | None:
    body = message.get("msg") or {}
    if not isinstance(body, dict):
        return None
    if message.get("topic") == "~/pose/info":
        return body if body.get("name") == model else None
    if message.get("topic") == "~/scene":
        models = body.get("model") or []
        if not isinstance(models, list):
            return None
        for item in models:
            if isinstance(item, dict) and item.get("name") == model:
                pose = item.get("pose") or {}
                if isinstance(pose, dict):
                    return {"name": model, **pose}
    return None


def poses(
    host: str,
    model: str,
    timeout: float,
    *,
    heartbeat: bool = False,
    deadline: float | None = None,
    connector: Any = None,
) -> Iterator[tuple[dict[str, Any], dict[str, Any]]]:
    """Yield `(pose, clock)` for one model from the gzweb bridge.

    The bridge broadcasts buffered messages to all clients, but explicit
    read-only subscriptions request the initial scene and latched world stats.
    The scene supplies a pose even when the vessel is stationary or the
    simulator is paused. Subsequent poses are filtered bridge-side to ~33 Hz
    and 1 cm of movement.

    `~/pose/info` carries no timestamp, so sim time is tracked separately from
    `~/world_stats` (~1 Hz). Differencing positions against wall clock instead
    produces nonsense whenever the sim stalls or bursts a backlog: a 0.0 m/s
    sample followed by a 14 m/s one for a vessel doing a steady 2.8.

    `clock` extrapolates sim time between those 1 Hz ticks using the real-time
    factor measured from consecutive stats messages. It goes None before that
    factor is known, when the sim is paused, or when the last tick is too old to
    trust; a None clock means no speed rather than a wrong one.

    Reconnects on drop, because pressing Reset in the control panel restarts the
    containers and we want the recording to survive it.
    """
    if connector is None:
        from websockets.sync.client import connect

        connector = connect

    while True:
        remaining = deadline - time.monotonic() if deadline is not None else None
        if remaining is not None and remaining <= 0:
            return
        try:
            open_timeout = timeout if remaining is None else min(timeout, remaining)
            with connector(
                f"ws://{host}:8080", open_timeout=open_timeout, max_size=None
            ) as socket:
                last_beat = 0.0
                clock_state = SimulationClock()
                for topic, message_type in (
                    ("~/scene", "scene"),
                    ("~/pose/info", "pose"),
                    ("~/world_stats", "world_stats"),
                ):
                    socket.send(
                        json.dumps(
                            {
                                "op": "subscribe",
                                "topic": topic,
                                "type": message_type,
                            }
                        )
                    )
                while True:
                    now_monotonic = time.monotonic()
                    remaining = deadline - now_monotonic if deadline is not None else None
                    if remaining is not None and remaining <= 0:
                        return
                    if heartbeat and now_monotonic - last_beat > 5.0:
                        # Exactly what gzweb's own client sends; only needed if
                        # the bridge is dropping silent clients.
                        socket.send(json.dumps({"op": "publish",
                                                "topic": "~/heartbeat",
                                                "msg": {"alive": 1}}))
                        last_beat = now_monotonic
                    recv_timeout = 1.0 if remaining is None else min(1.0, remaining)
                    try:
                        raw = socket.recv(timeout=recv_timeout)
                    except TimeoutError:
                        continue
                    try:
                        message = json.loads(raw)
                    except (TypeError, ValueError):
                        continue
                    if not isinstance(message, dict):
                        continue
                    topic = message.get("topic")
                    body = message.get("msg") or {}
                    if topic == "~/world_stats" and isinstance(body, dict):
                        clock_state.update(body, time.monotonic())
                        continue
                    pose = _pose_from_message(message, model)
                    if pose is None:
                        continue
                    yield pose, {
                        "clock": clock_state.current(time.monotonic()),
                        "sim_time": clock_state.sim_time,
                        "paused": clock_state.paused,
                    }
        except (OSError, TimeoutError) as exc:
            print(f"gzweb bridge disconnected ({exc}); retrying in 2 s", flush=True)
        except Exception as exc:  # websockets raises its own close exceptions
            print(f"gzweb bridge closed ({type(exc).__name__}); retrying in 2 s",
                  flush=True)
        remaining = deadline - time.monotonic() if deadline is not None else None
        if remaining is not None and remaining <= 0:
            return
        time.sleep(2.0 if remaining is None else min(2.0, remaining))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1", help="simulator address")
    parser.add_argument("--model", default="target_vessel", help="Gazebo model name")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after this long (0 = run until interrupted)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="minimum seconds between printed samples")
    parser.add_argument("--jsonl", help="append each sample to this file")
    parser.add_argument("--compare-url",
                        help="read estimates from this JSON endpoint; the WHITEOUT "
                             "dashboard uses http://127.0.0.1:8070/api/state")
    parser.add_argument("--compare-name",
                        help="track name when --compare-url returns a collection")
    parser.add_argument("--compare-timeout", type=float, default=2.0,
                        help="seconds allowed for each comparison API request")
    parser.add_argument("--no-true-scale", action="store_true",
                        help="site was built without --true-scale (k = 1)")
    parser.add_argument("--heartbeat", action="store_true",
                        help="send gzweb's own 5 s heartbeat; only if the bridge "
                             "drops a silent client")
    parser.add_argument("--confirm-network", action="store_true",
                        help="explicitly permit read-only simulator connections")
    args = parser.parse_args()
    if not args.confirm_network:
        parser.error("reading the simulator requires --confirm-network")
    if args.seconds < 0:
        parser.error("--seconds must be non-negative")
    if args.interval < 0:
        parser.error("--interval must be non-negative")
    if args.compare_timeout <= 0:
        parser.error("--compare-timeout must be positive")
    if args.compare_name and not args.compare_url:
        parser.error("--compare-name requires --compare-url")

    frame = WorldFrame(fetch_site(args.host), true_scale=not args.no_true_scale)
    print(f"site centre EPSG:3413 ({frame.cx:.1f}, {frame.cy:.1f}), "
          f"k={frame.k:.6f}, convergence {frame.convergence_deg:+.2f} deg")
    print(f"reading ~/pose/info for {args.model!r} from ws://{args.host}:8080")

    sink = open(args.jsonl, "a", encoding="utf-8") if args.jsonl else None
    started_monotonic = time.monotonic()
    deadline = started_monotonic + args.seconds if args.seconds else None
    last_emit = 0.0
    previous: tuple[float, float, float] | None = None
    estimate_poller = (
        EstimatePoller(
            args.compare_url,
            name=args.compare_name,
            timeout=args.compare_timeout,
        )
        if args.compare_url
        else None
    )
    if estimate_poller:
        estimate_poller.start()
    try:
        for body, clock_info in poses(args.host, args.model, timeout=10.0,
                                      heartbeat=args.heartbeat, deadline=deadline):
            now = time.time()
            now_monotonic = time.monotonic()
            sim_time = clock_info["sim_time"]
            paused = clock_info["paused"]
            if now_monotonic - last_emit < args.interval:
                continue
            last_emit = now_monotonic
            position = body.get("position") or {}
            x, y, z = (float(position.get(k, 0.0)) for k in ("x", "y", "z"))
            lat, lon = frame.to_latlon(x, y)
            bearing = frame.true_bearing_deg(quaternion_yaw(body.get("orientation") or {}))

            # Difference against sim time: the vessel moves on the sim's clock,
            # not ours. A None clock (paused, or stats too stale to trust) means
            # we decline to guess.
            clock = clock_info["clock"]
            speed = None
            if clock is not None and previous is not None:
                dt = clock - previous[2]
                # A gap means a dropped connection, a Reset, or a pause, and
                # differencing across it invents a speed. Report none instead.
                if 0 < dt < max(4.0, 3.0 * args.interval):
                    speed = math.hypot(x - previous[0], y - previous[1]) / dt
            previous = (x, y, clock) if clock is not None else None

            sample = {
                "wall_time": now,
                "sim_time_s": round(sim_time, 3) if sim_time is not None else None,
                "paused": paused,
                "model": args.model,
                "x_m": round(x, 2),
                "y_m": round(y, 2),
                "z_m": round(z, 2),
                "lat": round(lat, 7),
                "lon": round(lon, 7),
                "heading_true_deg": round(bearing, 1),
            }
            if speed is not None:
                sample["speed_mps"] = round(speed, 2)

            line = (f"truth  x={x:8.1f} y={y:8.1f}  {lat:.6f},{lon:.6f}  "
                    f"hdg {bearing:5.1f}")
            if speed is not None:
                line += f"  {speed:4.1f} m/s"
            if paused:
                line += "  [sim paused]"

            if estimate_poller:
                estimate, estimate_error, estimate_time = estimate_poller.snapshot()
                if estimate is not None:
                    estimated_lat, estimated_lon = estimate
                    error = haversine_m(lat, lon, estimated_lat, estimated_lon)
                    sample["reported_lat"] = estimated_lat
                    sample["reported_lon"] = estimated_lon
                    sample["error_m"] = round(error, 1)
                    if estimate_time is not None:
                        sample["estimate_age_s"] = round(max(0.0, now - estimate_time), 2)
                    label = args.compare_name or "estimate"
                    line += f"  | {label}: {error:7.1f} m off"
                elif estimate_error:
                    line += f"  | estimate API unavailable ({estimate_error})"
                else:
                    line += "  | awaiting estimate"

            print(line, flush=True)
            if sink:
                sink.write(json.dumps(sample) + "\n")
                sink.flush()

            if deadline is not None and now_monotonic >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if estimate_poller:
            estimate_poller.stop()
        if sink:
            sink.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
