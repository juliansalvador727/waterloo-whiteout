"""Small alpha-beta constant-velocity geographic tracker."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .models import GeoEstimate, Track


@dataclass(slots=True)
class ConstantVelocityTracker:
    track_id: str = "target-1"
    alpha: float = 0.65
    beta: float = 0.15
    _track: Track | None = None

    @property
    def track(self) -> Track | None:
        return self._track

    def update(self, observation: GeoEstimate) -> Track:
        if self._track is None:
            self._track = Track(
                self.track_id,
                observation.latitude,
                observation.longitude,
                0.0,
                0.0,
                observation.uncertainty_m,
                observation.timestamp,
            )
            return self._track

        prior = self._track
        dt = (observation.timestamp - prior.last_update).total_seconds()
        if dt <= 0:
            return prior
        meters_per_degree_lat = 111_319.490793
        meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(prior.latitude))
        predicted_lat = prior.latitude + prior.velocity_north_mps * dt / meters_per_degree_lat
        predicted_lon = prior.longitude + prior.velocity_east_mps * dt / meters_per_degree_lon
        north_residual = (observation.latitude - predicted_lat) * meters_per_degree_lat
        east_residual = (observation.longitude - predicted_lon) * meters_per_degree_lon
        self._track = Track(
            prior.track_id,
            predicted_lat + self.alpha * north_residual / meters_per_degree_lat,
            predicted_lon + self.alpha * east_residual / meters_per_degree_lon,
            prior.velocity_north_mps + self.beta * north_residual / dt,
            prior.velocity_east_mps + self.beta * east_residual / dt,
            max(0.1, self.alpha * observation.uncertainty_m + (1.0 - self.alpha) * prior.uncertainty_m),
            observation.timestamp,
        )
        return self._track

    def predict(self, seconds: float) -> Track | None:
        if self._track is None:
            return None
        if seconds < 0:
            raise ValueError("prediction interval cannot be negative")
        track = self._track
        meters_per_degree_lat = 111_319.490793
        meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(track.latitude))
        return Track(
            track.track_id,
            track.latitude + track.velocity_north_mps * seconds / meters_per_degree_lat,
            track.longitude + track.velocity_east_mps * seconds / meters_per_degree_lon,
            track.velocity_north_mps,
            track.velocity_east_mps,
            track.uncertainty_m + seconds,
            track.last_update,
        )

