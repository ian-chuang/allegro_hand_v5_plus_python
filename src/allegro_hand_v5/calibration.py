"""
Measured joint ranges for one physical Allegro Hand.

Encoder offsets and mechanical stops differ from hand to hand (the thumb can be
off by more than a radian), so the nominal URDF ranges rarely match reality.
This module loads measured ranges from JSON and converts between radians and
normalized 0..1 fractions of each joint's travel, so a pose written as
fractions is portable across hands.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

from allegro_hand_v5.can_driver import NUM_JOINTS

logger = logging.getLogger(__name__)

# Joint names, indexed exactly like get_positions().
JOINT_NAMES = [
    "Index Spread (0)",   "Index MCP (1)",   "Index PIP (2)",   "Index DIP (3)",
    "Middle Spread (4)",  "Middle MCP (5)",  "Middle PIP (6)",  "Middle DIP (7)",
    "Ring Spread (8)",    "Ring MCP (9)",    "Ring PIP (10)",   "Ring DIP (11)",
    "Thumb Rot (12)",     "Thumb MCP (13)",  "Thumb PIP (14)",  "Thumb DIP (15)",
]

FINGER_NAMES = ("index", "middle", "ring", "thumb")

# Nominal URDF ranges, used for joints this hand has not had measured.
DEFAULT_RANGES: Dict[int, Tuple[float, float]] = {
    0: (-0.30, 0.30),   1: (-0.01, 1.60),  2: (-0.07, 1.86),  3: (-0.02, 2.01),
    4: (-0.26, 0.26),   5: (-0.21, 1.79),  6: (-0.12, 1.86),  7: (-0.21, 1.85),
    8: (-0.26, 0.29),   9: (-0.21, 1.79),  10: (-0.12, 1.86), 11: (-0.21, 1.85),
    12: (0.00, 1.78),   13: (-0.26, 1.65), 14: (-0.05, 1.85), 15: (-0.09, 1.80),
}

CALIBRATION_DIRNAME = "calibration"


def _project_root() -> Path:
    """Project root for a src-layout install (.../src/allegro_hand_v5 -> ...)."""
    return Path(__file__).resolve().parent.parent.parent


def default_calibration_path(hand_type: str = "right") -> Optional[Path]:
    """First existing ``calibration/<hand_type>.json`` under cwd or the repo root."""
    hand_type = str(hand_type).lower()
    for base in (Path.cwd(), _project_root()):
        candidate = base / CALIBRATION_DIRNAME / f"{hand_type}.json"
        if candidate.is_file():
            return candidate
    return None


class HandCalibration:
    """
    Measured travel of each joint.

    ``low[i]`` maps to fraction 0.0 and ``high[i]`` to fraction 1.0. That
    ordering may be reversed (``low > high``) when a joint's encoder runs
    backwards; clipping always uses the true min/max, so the reversal only
    affects fraction direction.
    """

    def __init__(
        self,
        ranges: Optional[Mapping[int, Sequence[float]]] = None,
        hand_type: Optional[str] = None,
        calibrated_date: Optional[str] = None,
        source: Optional[Union[str, Path]] = None,
    ):
        ranges = dict(ranges or {})

        self.low = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.high = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.calibrated = np.zeros(NUM_JOINTS, dtype=bool)

        for i in range(NUM_JOINTS):
            if i in ranges:
                lo, hi = ranges[i]
                self.calibrated[i] = True
            elif str(i) in ranges:  # tolerate string keys straight from JSON
                lo, hi = ranges[str(i)]
                self.calibrated[i] = True
            else:
                lo, hi = DEFAULT_RANGES[i]
            self.low[i] = float(lo)
            self.high[i] = float(hi)

        self.hand_type = hand_type
        self.calibrated_date = calibrated_date
        self.source = Path(source) if source is not None else None

    # ==================== Constructors ====================

    @classmethod
    def default(cls, hand_type: Optional[str] = None) -> "HandCalibration":
        """Nominal URDF ranges; nothing measured."""
        return cls(ranges=None, hand_type=hand_type)

    @classmethod
    def from_dict(
        cls, data: Mapping, source: Optional[Union[str, Path]] = None
    ) -> "HandCalibration":
        """
        Build from a parsed JSON document. Both layouts are accepted::

            {"hand_type": ..., "joints": {"12": {"min": .., "max": ..}, ...}}
            {"12": [min, max], ...}
        """
        joints = data.get("joints", data)

        ranges: Dict[int, Tuple[float, float]] = {}
        for key, value in joints.items():
            try:
                idx = int(key)
            except (TypeError, ValueError):
                continue  # metadata key in a flat document
            if isinstance(value, Mapping):
                ranges[idx] = (float(value["min"]), float(value["max"]))
            else:
                lo, hi = value
                ranges[idx] = (float(lo), float(hi))

        return cls(
            ranges=ranges,
            hand_type=data.get("hand_type"),
            calibrated_date=data.get("calibration_date"),
            source=source,
        )

    @classmethod
    def load(
        cls,
        path: Optional[Union[str, Path]] = None,
        hand_type: str = "right",
        required: bool = False,
    ) -> "HandCalibration":
        """
        Load a calibration file, falling back to the URDF defaults.

        Args:
            path: Explicit file. If None, look for ``calibration/<hand_type>.json``.
            hand_type: Hand to look up when ``path`` is None.
            required: Raise FileNotFoundError instead of falling back.
        """
        if path is None:
            path = default_calibration_path(hand_type)

        if path is None:
            if required:
                raise FileNotFoundError(f"No calibration found for {hand_type} hand")
            logger.info("No calibration file for %s hand; using URDF defaults", hand_type)
            return cls.default(hand_type=hand_type)

        path = Path(path).expanduser()
        if not path.is_file():
            if required:
                raise FileNotFoundError(f"Calibration file not found: {path}")
            logger.warning("Calibration file not found: %s; using URDF defaults", path)
            return cls.default(hand_type=hand_type)

        with open(path) as f:
            data = json.load(f)

        cal = cls.from_dict(data, source=path)
        if cal.hand_type is None:
            cal.hand_type = hand_type
        return cal

    # ==================== Limits ====================

    @property
    def lower(self) -> np.ndarray:
        """Per-joint minimum in radians."""
        return np.minimum(self.low, self.high)

    @property
    def upper(self) -> np.ndarray:
        """Per-joint maximum in radians."""
        return np.maximum(self.low, self.high)

    @property
    def center(self) -> np.ndarray:
        return 0.5 * (self.lower + self.upper)

    def limits(self, joint_idx: int) -> Tuple[float, float]:
        """(min, max) in radians for one joint."""
        return float(self.lower[joint_idx]), float(self.upper[joint_idx])

    def clip(self, positions: Union[Sequence[float], np.ndarray]) -> np.ndarray:
        """Clip a 16-joint target into the measured range."""
        positions = np.asarray(positions, dtype=np.float64)
        if positions.shape[-1] != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} positions, got {positions.shape[-1]}")
        return np.clip(positions, self.lower, self.upper)

    # ==================== Fractions <-> radians ====================

    def from_fractions(self, fractions: Union[Sequence[float], np.ndarray]) -> np.ndarray:
        """16 fractions (0 = low, 1 = high) -> radians, clipped to the range."""
        fractions = np.asarray(fractions, dtype=np.float64)
        if fractions.shape[-1] != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} fractions, got {fractions.shape[-1]}")
        return self.clip(self.low + fractions * (self.high - self.low))

    def to_fractions(self, positions: Union[Sequence[float], np.ndarray]) -> np.ndarray:
        """Radians -> fractions (0 = low, 1 = high)."""
        positions = np.asarray(positions, dtype=np.float64)
        span = self.high - self.low
        safe = np.where(np.abs(span) < 1e-9, 1.0, span)
        return (positions - self.low) / safe

    def joint(self, joint_idx: int, fraction: float) -> float:
        """Radians at ``fraction`` of one joint's travel."""
        lo, hi = self.low[joint_idx], self.high[joint_idx]
        value = lo + float(fraction) * (hi - lo)
        return float(np.clip(value, self.lower[joint_idx], self.upper[joint_idx]))

    # ==================== Display ====================

    def summary(self) -> str:
        """Formatted table of the ranges."""
        origin = str(self.source) if self.source else "defaults (URDF)"
        lines = [f"Calibration ({self.hand_type or 'unknown'} hand) from {origin}"]
        if self.calibrated_date:
            lines.append(f"Recorded: {self.calibrated_date}")
        lines.append(f"{'Joint':<22}{'Min':>9}{'Max':>9}{'Range':>9}  Source")
        lines.append("-" * 62)
        for i in range(NUM_JOINTS):
            lo, hi = self.limits(i)
            tag = "measured" if self.calibrated[i] else "default"
            lines.append(f"{JOINT_NAMES[i]:<22}{lo:>+9.3f}{hi:>+9.3f}{hi - lo:>9.3f}  {tag}")
        return "\n".join(lines)

    def __repr__(self) -> str:
        n = int(self.calibrated.sum())
        return (
            f"HandCalibration(hand_type={self.hand_type!r}, "
            f"measured_joints={n}/{NUM_JOINTS}, source={str(self.source)!r})"
        )


def load_calibration(
    calibration: Union[None, bool, str, Path, HandCalibration] = None,
    hand_type: str = "right",
) -> Optional[HandCalibration]:
    """
    Coerce the several ways of specifying a calibration into an object.

    Returns None when ``calibration`` is None or False (i.e. clipping disabled).
    """
    if calibration is None or calibration is False:
        return None
    if isinstance(calibration, HandCalibration):
        return calibration
    if calibration is True:
        return HandCalibration.load(hand_type=hand_type)
    return HandCalibration.load(path=calibration, hand_type=hand_type)
