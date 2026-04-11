"""
Session logger for post-processing and replay.

Writes one JSON object per line with timestamped GNSS events.
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from threading import RLock
from typing import Any, Optional

import numpy as np


def _to_jsonable(value: Any) -> Any:
    """Convert nested objects to JSON-safe values."""
    if is_dataclass(value):
        return _to_jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): _to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if hasattr(value, "name") and hasattr(value, "value"):
        # Supports enums from constants.py without importing here.
        return value.name
    return value


class SessionLogger:
    """Thread-safe JSONL session logger."""

    def __init__(self):
        self._file = None
        self._path: Optional[Path] = None
        self._lock = RLock()
        self._started_at = 0.0

    @property
    def is_active(self) -> bool:
        return self._file is not None

    @property
    def path(self) -> Optional[str]:
        return str(self._path) if self._path else None

    def start(self, file_path: str) -> bool:
        with self._lock:
            if self._file:
                return False

            path = Path(file_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = path.open("a", encoding="utf-8")
            self._path = path
            self._started_at = time.time()

            self.log_event("session_start", {"path": str(path)})
            return True

    def stop(self):
        with self._lock:
            if not self._file:
                return
            self.log_event("session_stop", {"duration_s": time.time() - self._started_at})
            self._file.close()
            self._file = None

    def log_event(self, event_type: str, payload: dict[str, Any]):
        with self._lock:
            if not self._file:
                return

            row = {
                "t_unix": time.time(),
                "event": event_type,
                "payload": _to_jsonable(payload),
            }
            self._file.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._file.flush()
