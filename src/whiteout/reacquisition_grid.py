"""Deterministic spatial evidence grid for reacquisition planning."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .models import GeoEstimate


@dataclass(frozen=True, slots=True)
class GridCell:
    row: int
    column: int
    south: float
    west: float
    north: float
    east: float
    weight: float
    evidence_latitude: float
    evidence_longitude: float


@dataclass(slots=True)
class _CellEvidence:
    weight: float
    latitude: float
    longitude: float
    updated_s: float


@dataclass(slots=True)
class WeightedReacquisitionGrid:
    south: float
    west: float
    north: float
    east: float
    rows: int = 12
    columns: int = 12
    half_life_s: float = 30.0
    neighborhood_cells: int = 2
    detection_weight: float = 1.0
    prediction_weight: float = 4.0
    maximum_cell_weight: float = 8.0
    _cells: dict[tuple[int, int], _CellEvidence] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.south >= self.north or self.west >= self.east:
            raise ValueError("reacquisition grid bounds must have positive area")
        if self.rows <= 0 or self.columns <= 0:
            raise ValueError("reacquisition grid dimensions must be positive")
        if self.half_life_s <= 0 or self.neighborhood_cells < 0:
            raise ValueError("reacquisition grid decay and neighborhood are invalid")
        if (
            self.detection_weight <= 0
            or self.prediction_weight <= 0
            or self.maximum_cell_weight <= 0
        ):
            raise ValueError("reacquisition grid weights must be positive")

    def cell_index(self, latitude: float, longitude: float) -> tuple[int, int]:
        if not self.contains(latitude, longitude):
            raise ValueError("position is outside reacquisition grid")
        row = min(
            self.rows - 1,
            int((latitude - self.south) / (self.north - self.south) * self.rows),
        )
        column = min(
            self.columns - 1,
            int((longitude - self.west) / (self.east - self.west) * self.columns),
        )
        return row, column

    def contains(self, latitude: float, longitude: float) -> bool:
        return self.south <= latitude <= self.north and self.west <= longitude <= self.east

    def observe(self, estimate: GeoEstimate, now_s: float) -> None:
        if not self.contains(estimate.latitude, estimate.longitude):
            return
        key = self.cell_index(estimate.latitude, estimate.longitude)
        previous = self._cells.get(key)
        previous_weight = self._decayed_weight(previous, now_s) if previous else 0.0
        combined_weight = previous_weight + self.detection_weight
        if previous is None or previous_weight <= 1e-12:
            latitude = estimate.latitude
            longitude = estimate.longitude
        else:
            latitude = (
                previous.latitude * previous_weight
                + estimate.latitude * self.detection_weight
            ) / combined_weight
            longitude = (
                previous.longitude * previous_weight
                + estimate.longitude * self.detection_weight
            ) / combined_weight
        self._cells[key] = _CellEvidence(
            min(self.maximum_cell_weight, combined_weight),
            latitude,
            longitude,
            float(now_s),
        )

    def weighted_target(self, predicted: GeoEstimate, now_s: float) -> GeoEstimate:
        """Bias a prediction toward recent detection evidence in nearby cells."""
        if not self.contains(predicted.latitude, predicted.longitude):
            return predicted
        predicted_row, predicted_column = self.cell_index(
            predicted.latitude, predicted.longitude
        )
        total_weight = self.prediction_weight
        latitude_sum = predicted.latitude * total_weight
        longitude_sum = predicted.longitude * total_weight
        for (row, column), evidence in self._cells.items():
            row_distance = abs(row - predicted_row)
            column_distance = abs(column - predicted_column)
            if max(row_distance, column_distance) > self.neighborhood_cells:
                continue
            weight = self._decayed_weight(evidence, now_s)
            if weight <= 1e-6:
                continue
            cell_distance = math.hypot(row_distance, column_distance)
            local_weight = weight / (1.0 + cell_distance)
            total_weight += local_weight
            latitude_sum += evidence.latitude * local_weight
            longitude_sum += evidence.longitude * local_weight
        return GeoEstimate(
            latitude_sum / total_weight,
            longitude_sum / total_weight,
            predicted.uncertainty_m,
            predicted.timestamp,
        )

    def cells(self, now_s: float) -> tuple[GridCell, ...]:
        """Return non-empty cells for dashboarding, recording, or inspection."""
        latitude_step = (self.north - self.south) / self.rows
        longitude_step = (self.east - self.west) / self.columns
        result: list[GridCell] = []
        for (row, column), evidence in sorted(self._cells.items()):
            weight = self._decayed_weight(evidence, now_s)
            if weight <= 1e-6:
                continue
            result.append(GridCell(
                row,
                column,
                self.south + row * latitude_step,
                self.west + column * longitude_step,
                self.south + (row + 1) * latitude_step,
                self.west + (column + 1) * longitude_step,
                weight,
                evidence.latitude,
                evidence.longitude,
            ))
        return tuple(result)

    def _decayed_weight(self, evidence: _CellEvidence, now_s: float) -> float:
        age_s = max(0.0, float(now_s) - evidence.updated_s)
        return evidence.weight * math.pow(0.5, age_s / self.half_life_s)
