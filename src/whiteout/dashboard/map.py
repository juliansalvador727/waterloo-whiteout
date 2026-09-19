"""Map configuration derived from the simulator's existing site endpoint."""

from __future__ import annotations

import json
import urllib.request
from typing import Any, Callable


class MapAssetsUnavailable(RuntimeError):
    pass


class MapAssets:
    """Expose browser map sources without requiring simulator-side changes."""

    IMAGERY_TILES = (
        "https://server.arcgisonline.com/ArcGIS/rest/services/"
        "World_Imagery/MapServer/tile/{z}/{y}/{x}"
    )
    TERRAIN_TILES = "https://tiles.mapterhorn.com/{z}/{x}/{y}.webp"

    def __init__(
        self,
        sim_host: str,
        *,
        opener: Callable[..., object] = urllib.request.urlopen,
    ) -> None:
        self.site_url = f"http://{sim_host}:8090/api/site"
        self._opener = opener

    def _site(self) -> dict[str, Any]:
        try:
            with self._opener(self.site_url, timeout=5.0) as response:
                payload = json.loads(response.read())  # type: ignore[attr-defined]
        except (OSError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
            raise MapAssetsUnavailable("simulator site metadata is unavailable") from exc
        if not isinstance(payload, dict):
            raise MapAssetsUnavailable("simulator site metadata is invalid")
        return payload

    def config(self) -> dict[str, Any]:
        try:
            from pyproj import Transformer  # type: ignore[import-not-found]
        except ImportError as exc:
            raise MapAssetsUnavailable("map projection dependency is unavailable") from exc

        site = self._site()
        try:
            centre = site["centre"]
            bounds = site["bounds3413"]
            projected_bounds = (
                float(bounds["xmin"]),
                float(bounds["ymin"]),
                float(bounds["xmax"]),
                float(bounds["ymax"]),
            )
            center = [float(centre["lon"]), float(centre["lat"])]
        except (KeyError, TypeError, ValueError) as exc:
            raise MapAssetsUnavailable("simulator site metadata is invalid") from exc

        inverse = Transformer.from_crs("EPSG:3413", "EPSG:4326", always_xy=True)
        xmin, ymin, xmax, ymax = projected_bounds
        boundary_projected = (
            inverse.transform(xmin, ymin),
            inverse.transform(xmax, ymin),
            inverse.transform(xmax, ymax),
            inverse.transform(xmin, ymax),
            inverse.transform(xmin, ymin),
        )
        boundary = [[float(point[0]), float(point[1])] for point in boundary_projected]
        longitudes = [point[0] for point in boundary]
        latitudes = [point[1] for point in boundary]
        nominal_extent = float(site.get("extent_m", min(xmax - xmin, ymax - ymin)))
        return {
            "available": True,
            "name": str(site.get("name", "site")),
            "center": center,
            "bounds": [min(longitudes), min(latitudes), max(longitudes), max(latitudes)],
            "boundary": boundary,
            "dimensions_m": {
                "width": nominal_extent,
                "height": nominal_extent,
                "area_km2": nominal_extent * nominal_extent / 1_000_000.0,
            },
            "minzoom": 3,
            "maxzoom": 17,
            "imagery_tiles": [self.IMAGERY_TILES],
            "terrain_tiles": [self.TERRAIN_TILES],
            "terrain_encoding": "terrarium",
        }
