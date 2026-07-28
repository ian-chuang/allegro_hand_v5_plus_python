"""
Per-hand joint calibration: measured travel plus a homing offset.

Encoder zeros and mechanical stops differ from hand to hand — on the reference
V5 Plus the thumb sits more than a radian away from its nominal range — so the
manual's ROM table rarely matches what you measure. A calibration file records,
for one physical hand:

* ``min`` / ``max`` — the raw encoder angles seen at each joint's travel limits;
* ``offset`` — a homing offset added to every raw reading, 0 until you set it.

Everything the driver reports and accepts is in the offset frame:

    position = raw + offset
    limits   = (min + offset, max + offset)

so re-editing an offset shifts a joint's zero and its limits together. Files
live in ``calibration_data/<serial>.json`` and are written by
``examples/calibrate.py``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path
from typing import Optional, Sequence, Union

import numpy as np

from allegro_hand_v5.protocol import JOINT_NAMES, NUM_JOINTS

#: Directory holding the shipped calibrations, one JSON file per serial number.
CALIBRATION_DIR = Path(__file__).resolve().parent / "calibration_data"

#: Nominal range of motion in degrees, from manual section 15.1 (Plus values).
#: Used when a hand has no calibration file. Per finger: MCP-1, MCP-2, PIP, DIP;
#: thumb: CMC-1, CMC-2, MP, IP.
NOMINAL_RANGE_DEG = (
    (-16.0, 16.0), (-5.0, 110.0), (-3.0, 102.0), (-5.0, 105.0),
) * 3 + (
    (0.0, 105.0), (-6.0, 107.0), (-5.0, 106.0), (-4.0, 104.0),
)

NOMINAL_MIN = np.radians([lo for lo, _ in NOMINAL_RANGE_DEG])
NOMINAL_MAX = np.radians([hi for _, hi in NOMINAL_RANGE_DEG])


def _as16(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (NUM_JOINTS,):
        raise ValueError(f"{name} must have {NUM_JOINTS} values, got {array.shape}")
    return array


@dataclass
class Calibration:
    """
    Measured travel and homing offset of one physical hand, in radians.

    Attributes:
        min: (16,) raw encoder angle at each joint's low limit.
        max: (16,) raw encoder angle at each joint's high limit.
        offset: (16,) homing offset added to every raw reading.
        serial: Serial number of the hand this was measured on.
        date: ISO date it was recorded.
        path: File it was loaded from, if any.
    """

    min: np.ndarray = field(default_factory=lambda: NOMINAL_MIN.copy())
    max: np.ndarray = field(default_factory=lambda: NOMINAL_MAX.copy())
    offset: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    serial: str = ""
    date: str = ""
    path: Optional[Path] = None

    def __post_init__(self):
        self.min = _as16(self.min, "min")
        self.max = _as16(self.max, "max")
        self.offset = _as16(self.offset, "offset")

    # ==================== Use ====================

    @property
    def lower(self) -> np.ndarray:
        """Per-joint lower limit in the offset frame."""
        return np.minimum(self.min, self.max) + self.offset

    @property
    def upper(self) -> np.ndarray:
        """Per-joint upper limit in the offset frame."""
        return np.maximum(self.min, self.max) + self.offset

    @property
    def center(self) -> np.ndarray:
        """Midpoint of each joint's travel, in the offset frame."""
        return 0.5 * (self.lower + self.upper)

    def apply(self, raw: Sequence[float]) -> np.ndarray:
        """Raw encoder angles -> offset frame. This is what the driver reports."""
        return _as16(raw, "positions") + self.offset

    def clip(self, positions: Sequence[float]) -> np.ndarray:
        """Clip a 16-joint target into the calibrated range."""
        return np.clip(_as16(positions, "positions"), self.lower, self.upper)

    def denormalize(self, fractions: Sequence[float]) -> np.ndarray:
        """16 fractions (0 = lower limit, 1 = upper) -> radians."""
        return self.lower + _as16(fractions, "fractions") * (self.upper - self.lower)

    def normalize(self, positions: Sequence[float]) -> np.ndarray:
        """Radians -> fractions of travel (0 = lower limit, 1 = upper)."""
        span = self.upper - self.lower
        return (_as16(positions, "positions") - self.lower) / np.where(span > 1e-9, span, 1.0)

    # ==================== Files ====================

    @staticmethod
    def path_for(serial: str) -> Path:
        """Where the calibration for a serial number lives."""
        return CALIBRATION_DIR / f"{serial}.json"

    @classmethod
    def for_serial(cls, serial: str) -> "Calibration":
        """Load `calibration_data/<serial>.json`, or nominal limits if absent."""
        path = cls.path_for(serial)
        return cls.load(path) if path.is_file() else cls(serial=serial)

    @classmethod
    def load(cls, path: Union[str, Path]) -> "Calibration":
        """Load a calibration file. Raises FileNotFoundError if it is missing."""
        path = Path(path).expanduser()
        with open(path) as f:
            data = json.load(f)
        cal = cls.from_dict(data)
        cal.path = path
        return cal

    @classmethod
    def from_dict(cls, data: dict) -> "Calibration":
        """Build from a parsed calibration document."""
        cal = cls(serial=data.get("serial", ""), date=data.get("date", ""))
        for name, values in data.get("joints", {}).items():
            if name not in JOINT_NAMES:
                raise ValueError(f"unknown joint name {name!r} in calibration")
            i = JOINT_NAMES.index(name)
            cal.min[i] = float(values["min"])
            cal.max[i] = float(values["max"])
            cal.offset[i] = float(values.get("offset", 0.0))
        return cal

    def to_dict(self) -> dict:
        """The document written to disk. All angles in radians."""
        return {
            "serial": self.serial,
            "date": self.date or date.today().isoformat(),
            "units": "radians",
            "note": "min/max are raw encoder angles; offset is added to every reading",
            "joints": {
                name: {
                    "min": round(float(self.min[i]), 5),
                    "max": round(float(self.max[i]), 5),
                    "offset": round(float(self.offset[i]), 5),
                }
                for i, name in enumerate(JOINT_NAMES)
            },
        }

    def save(self, path: Union[str, Path, None] = None) -> Path:
        """
        Write to `path`, or to `calibration_data/<serial>.json` by default.

        Returns the path written.
        """
        if path is None:
            if not self.serial:
                raise ValueError("no serial number: pass an explicit path")
            path = self.path_for(self.serial)
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2) + "\n")
        self.path = path
        return path

    # ==================== Display ====================

    def summary(self) -> str:
        """Formatted table of the calibrated range, in degrees."""
        lines = [
            f"Calibration for {self.serial or 'unknown hand'}"
            f"{f' ({self.date})' if self.date else ''}"
            f"  from {self.path or 'nominal limits'}",
            f"{'joint':<14}{'min':>9}{'max':>9}{'offset':>9}   (degrees)",
            "-" * 50,
        ]
        for i, name in enumerate(JOINT_NAMES):
            lines.append(
                f"{name:<14}{np.degrees(self.lower[i]):>9.1f}"
                f"{np.degrees(self.upper[i]):>9.1f}{np.degrees(self.offset[i]):>9.1f}"
            )
        return "\n".join(lines)

    def __repr__(self) -> str:
        return (f"Calibration(serial={self.serial!r}, date={self.date!r}, "
                f"path={str(self.path) if self.path else None!r})")


def load_calibration(
    calibration: Union[None, bool, str, Path, Calibration] = True,
    serial: str = "",
) -> Optional[Calibration]:
    """
    Coerce the several ways of specifying a calibration into an object.

    Args:
        calibration: True to look up `serial` in `calibration_data/`, a path to
            load, a `Calibration` to use as is, or None/False to disable it.
        serial: Serial number, used when `calibration` is True.

    Returns:
        A Calibration, or None if calibration is disabled.
    """
    if calibration is None or calibration is False:
        return None
    if isinstance(calibration, Calibration):
        return calibration
    if calibration is True:
        return Calibration.for_serial(serial)
    return Calibration.load(calibration)
