"""Ordered constant-velocity Kalman tracking in a local tangent plane."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timedelta

from .models import GeoEstimate, Track


_METERS_PER_DEGREE_LAT = 111_319.490793
_CHI_SQUARE_2D_99 = 9.21034037197618


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [[sum(a * b for a, b in zip(row, column, strict=True)) for column in zip(*right, strict=True)] for row in left]


def _transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(column) for column in zip(*matrix, strict=True)]


@dataclass(slots=True)
class ConstantVelocityTracker:
    track_id: str = "target-1"
    # Retained for constructor compatibility with the former alpha-beta tracker.
    alpha: float = 0.65
    beta: float = 0.15
    process_acceleration_mps2: float = 2.0
    gate_threshold: float = _CHI_SQUARE_2D_99
    maximum_speed_mps: float = 20.0
    _track: Track | None = None
    _state: list[float] | None = None
    _covariance: list[list[float]] | None = None
    _origin_latitude: float | None = None
    _origin_longitude: float | None = None
    last_rejection_reason: str | None = field(default=None, init=False)

    @property
    def track(self) -> Track | None:
        return self._track

    def update(self, observation: GeoEstimate) -> Track:
        if self._track is None:
            sigma2 = max(0.1, observation.uncertainty_m) ** 2
            self._origin_latitude = observation.latitude
            self._origin_longitude = observation.longitude
            self._state = [0.0, 0.0, 0.0, 0.0]
            self._covariance = [
                [sigma2, 0.0, 0.0, 0.0],
                [0.0, sigma2, 0.0, 0.0],
                [0.0, 0.0, 400.0, 0.0],
                [0.0, 0.0, 0.0, 400.0],
            ]
            self._track = Track(
                self.track_id,
                observation.latitude,
                observation.longitude,
                0.0,
                0.0,
                max(0.1, observation.uncertainty_m),
                observation.timestamp,
            )
            self.last_rejection_reason = None
            return self._track

        prior = self._track
        dt = (observation.timestamp - prior.last_update).total_seconds()
        if dt <= 0:
            self.last_rejection_reason = "observation timestamp is not ordered"
            return prior

        assert self._state is not None and self._covariance is not None
        predicted_state, predicted_covariance = self._predict_state(dt)
        measured_north, measured_east = self._to_local(observation.latitude, observation.longitude)
        residual_north = measured_north - predicted_state[0]
        residual_east = measured_east - predicted_state[1]
        measurement_variance = max(0.1, observation.uncertainty_m) ** 2
        s00 = predicted_covariance[0][0] + measurement_variance
        s01 = predicted_covariance[0][1]
        s10 = predicted_covariance[1][0]
        s11 = predicted_covariance[1][1] + measurement_variance
        determinant = s00 * s11 - s01 * s10
        if determinant <= 0:
            self.last_rejection_reason = "invalid innovation covariance"
            return prior
        inv_s = [[s11 / determinant, -s01 / determinant], [-s10 / determinant, s00 / determinant]]
        mahalanobis2 = (
            residual_north * (inv_s[0][0] * residual_north + inv_s[0][1] * residual_east)
            + residual_east * (inv_s[1][0] * residual_north + inv_s[1][1] * residual_east)
        )
        if mahalanobis2 > self.gate_threshold:
            self.last_rejection_reason = "observation rejected by 99 percent innovation gate"
            return prior

        pht = [[row[0], row[1]] for row in predicted_covariance]
        gain = _matmul(pht, inv_s)
        updated_state = [
            predicted_state[index] + gain[index][0] * residual_north + gain[index][1] * residual_east
            for index in range(4)
        ]
        if math.hypot(updated_state[2], updated_state[3]) > self.maximum_speed_mps:
            self.last_rejection_reason = "observation implies speed above 20 m/s"
            return prior

        kh = [[gain[row][0], gain[row][1], 0.0, 0.0] for row in range(4)]
        identity_minus_kh = [
            [(1.0 if row == column else 0.0) - kh[row][column] for column in range(4)]
            for row in range(4)
        ]
        left = _matmul(_matmul(identity_minus_kh, predicted_covariance), _transpose(identity_minus_kh))
        krkt = [
            [measurement_variance * sum(gain[row][axis] * gain[column][axis] for axis in range(2)) for column in range(4)]
            for row in range(4)
        ]
        updated_covariance = [[left[row][column] + krkt[row][column] for column in range(4)] for row in range(4)]
        self._state = updated_state
        self._covariance = updated_covariance
        self._track = self._as_track(updated_state, updated_covariance, observation.timestamp)
        self.last_rejection_reason = None
        return self._track

    def predict(self, seconds: float) -> Track | None:
        if self._track is None:
            return None
        if seconds < 0:
            raise ValueError("prediction interval cannot be negative")
        state, covariance = self._predict_state(seconds)
        return self._as_track(state, covariance, self._track.last_update + timedelta(seconds=seconds))

    def _predict_state(self, seconds: float) -> tuple[list[float], list[list[float]]]:
        assert self._state is not None and self._covariance is not None
        dt = seconds
        transition = [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        state = [self._state[0] + dt * self._state[2], self._state[1] + dt * self._state[3], self._state[2], self._state[3]]
        q = self.process_acceleration_mps2**2
        process_noise = [
            [q * dt**4 / 4.0, 0.0, q * dt**3 / 2.0, 0.0],
            [0.0, q * dt**4 / 4.0, 0.0, q * dt**3 / 2.0],
            [q * dt**3 / 2.0, 0.0, q * dt**2, 0.0],
            [0.0, q * dt**3 / 2.0, 0.0, q * dt**2],
        ]
        propagated = _matmul(_matmul(transition, self._covariance), _transpose(transition))
        covariance = [[propagated[row][column] + process_noise[row][column] for column in range(4)] for row in range(4)]
        return state, covariance

    def _to_local(self, latitude: float, longitude: float) -> tuple[float, float]:
        assert self._origin_latitude is not None and self._origin_longitude is not None
        longitude_scale = _METERS_PER_DEGREE_LAT * math.cos(math.radians(self._origin_latitude))
        return (
            (latitude - self._origin_latitude) * _METERS_PER_DEGREE_LAT,
            (longitude - self._origin_longitude) * longitude_scale,
        )

    def _as_track(self, state: list[float], covariance: list[list[float]], timestamp: datetime) -> Track:
        assert self._origin_latitude is not None and self._origin_longitude is not None
        longitude_scale = _METERS_PER_DEGREE_LAT * math.cos(math.radians(self._origin_latitude))
        uncertainty = math.sqrt(max(0.01, covariance[0][0], covariance[1][1]))
        return Track(
            self.track_id,
            self._origin_latitude + state[0] / _METERS_PER_DEGREE_LAT,
            self._origin_longitude + state[1] / longitude_scale,
            state[2],
            state[3],
            uncertainty,
            timestamp,
        )

