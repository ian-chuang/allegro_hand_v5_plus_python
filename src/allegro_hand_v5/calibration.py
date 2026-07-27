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
import os
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

from allegro_hand_v5.protocol import FINGER_NAMES, JOINT_LABELS, NUM_JOINTS

logger = logging.getLogger(__name__)

# Nominal URDF ranges, used for joints this hand has not had measured.
DEFAULT_RANGES: Dict[int, Tuple[float, float]] = {
    0: (-0.30, 0.30),   1: (-0.01, 1.60),  2: (-0.07, 1.86),  3: (-0.02, 2.01),
    4: (-0.26, 0.26),   5: (-0.21, 1.79),  6: (-0.12, 1.86),  7: (-0.21, 1.85),
    8: (-0.26, 0.29),   9: (-0.21, 1.79),  10: (-0.12, 1.86), 11: (-0.21, 1.85),
    12: (0.00, 1.78),   13: (-0.26, 1.65), 14: (-0.05, 1.85), 15: (-0.09, 1.80),
}

#: Calibrations shipped with the package, named `<handedness>_<hardware type>.json`.
BUNDLED_DIR = Path(__file__).resolve().parent / "calibration_data"

#: Directory name searched under the working directory, so a project can
#: override a bundled calibration without editing the installed package.
CALIBRATION_DIRNAME = "calibration"

#: Environment variable holding an explicit calibration file path.
ENV_VAR = "ALLEGRO_CALIBRATION"


def calibration_filenames(handedness: str = "right", hardware_type: Optional[str] = None) -> list:
    """
    Candidate file names, most specific first.

    ``("right", "B")`` gives ``["right_B.json", "right.json"]`` — the hand-type
    specific file if there is one, otherwise a handedness-only file.
    """
    handedness = str(handedness).lower()
    names = []
    if hardware_type:
        names.append(f"{handedness}_{str(hardware_type).upper()}.json")
    names.append(f"{handedness}.json")
    return names


def calibration_search_paths(
    handedness: str = "right", hardware_type: Optional[str] = None
) -> list:
    """Every location checked for a calibration, in priority order."""
    paths = []

    env = os.environ.get(ENV_VAR)
    if env:
        paths.append(Path(env).expanduser())

    names = calibration_filenames(handedness, hardware_type)
    # A local ./calibration/ directory wins over the bundled files, so a
    # measurement for your own hand overrides the shipped one.
    for name in names:
        paths.append(Path.cwd() / CALIBRATION_DIRNAME / name)
    for name in names:
        paths.append(BUNDLED_DIR / name)

    seen, unique = set(), []
    for p in paths:
        if str(p) not in seen:
            seen.add(str(p))
            unique.append(p)
    return unique


def default_calibration_path(
    handedness: str = "right", hardware_type: Optional[str] = None
) -> Optional[Path]:
    """First existing calibration file for this hand, or None."""
    for path in calibration_search_paths(handedness, hardware_type):
        if path.is_file():
            return path
    return None


def available_calibrations() -> list:
    """Every calibration bundled with the package."""
    return sorted(BUNDLED_DIR.glob("*.json")) if BUNDLED_DIR.is_dir() else []


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
        handedness: Optional[str] = None,
        hardware_type: Optional[str] = None,
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

        self.handedness = handedness
        self.hardware_type = hardware_type
        self.calibrated_date = calibrated_date
        self.source = Path(source) if source is not None else None

    # ==================== Constructors ====================

    @classmethod
    def default(
        cls, handedness: Optional[str] = None, hardware_type: Optional[str] = None
    ) -> "HandCalibration":
        """Nominal URDF ranges; nothing measured."""
        return cls(ranges=None, handedness=handedness, hardware_type=hardware_type)

    @classmethod
    def from_dict(
        cls, data: Mapping, source: Optional[Union[str, Path]] = None
    ) -> "HandCalibration":
        """
        Build from a parsed JSON document. Both layouts are accepted::

            {"handedness": "right", "hardware_type": "B",
             "joints": {"12": {"min": .., "max": ..}, ...}}
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
            # "hand_type" is the key older files used for handedness.
            handedness=data.get("handedness") or data.get("hand_type"),
            hardware_type=data.get("hardware_type"),
            calibrated_date=data.get("calibration_date"),
            source=source,
        )

    @classmethod
    def load(
        cls,
        path: Optional[Union[str, Path]] = None,
        handedness: str = "right",
        hardware_type: Optional[str] = None,
        required: bool = False,
    ) -> "HandCalibration":
        """
        Load a calibration file, falling back to the URDF defaults.

        Args:
            path: Explicit file. If None, search `calibration_search_paths()`:
                $ALLEGRO_CALIBRATION, then ./calibration/, then the copies
                bundled with the package.
            handedness: "left" or "right", used when `path` is None.
            hardware_type: "A" or "B". Selects `<handedness>_<type>.json` in
                preference to `<handedness>.json`.
            required: Raise FileNotFoundError instead of falling back.
        """
        label = f"{handedness}{'/' + hardware_type if hardware_type else ''}"

        if path is None:
            path = default_calibration_path(handedness, hardware_type)

        if path is None:
            if required:
                raise FileNotFoundError(
                    f"No calibration for the {label} hand. Searched: "
                    + ", ".join(str(p) for p in calibration_search_paths(handedness, hardware_type))
                )
            logger.info("No calibration file for the %s hand; using URDF defaults", label)
            return cls.default(handedness=handedness, hardware_type=hardware_type)

        path = Path(path).expanduser()
        if not path.is_file():
            if required:
                raise FileNotFoundError(f"Calibration file not found: {path}")
            logger.warning("Calibration file not found: %s; using URDF defaults", path)
            return cls.default(handedness=handedness, hardware_type=hardware_type)

        with open(path) as f:
            data = json.load(f)

        cal = cls.from_dict(data, source=path)
        if cal.handedness is None:
            cal.handedness = handedness
        if cal.hardware_type is None:
            cal.hardware_type = hardware_type
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
        label = self.handedness or "unknown"
        if self.hardware_type:
            label += f" / type {self.hardware_type}"
        lines = [f"Calibration ({label} hand) from {origin}"]
        if self.calibrated_date:
            lines.append(f"Recorded: {self.calibrated_date}")
        lines.append(f"{'Joint':<22}{'Min':>9}{'Max':>9}{'Range':>9}  Source")
        lines.append("-" * 62)
        for i in range(NUM_JOINTS):
            lo, hi = self.limits(i)
            tag = "measured" if self.calibrated[i] else "default"
            lines.append(f"{JOINT_LABELS[i]:<22}{lo:>+9.3f}{hi:>+9.3f}{hi - lo:>9.3f}  {tag}")
        return "\n".join(lines)

    def __repr__(self) -> str:
        n = int(self.calibrated.sum())
        return (
            f"HandCalibration(handedness={self.handedness!r}, "
            f"hardware_type={self.hardware_type!r}, "
            f"measured_joints={n}/{NUM_JOINTS}, source={str(self.source)!r})"
        )


def load_calibration(
    calibration: Union[None, bool, str, Path, HandCalibration] = None,
    handedness: str = "right",
    hardware_type: Optional[str] = None,
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
        return HandCalibration.load(handedness=handedness, hardware_type=hardware_type)
    return HandCalibration.load(
        path=calibration, handedness=handedness, hardware_type=hardware_type
    )
