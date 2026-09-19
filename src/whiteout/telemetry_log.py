"""Append-only JSONL event logging."""

from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


class JsonlLogger:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def write(self, event: str, payload: Any) -> None:
        value = asdict(payload) if is_dataclass(payload) else payload
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "event": event,
            "payload": value,
        }
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, default=str, separators=(",", ":")) + "\n")
