"""
PD gain profiles for the Allegro Hand V5.

The two BHand profiles are the gains WONIK's `libBHand.so` actually uses, read
back out of a live instance. They are the reference point: `BHAND_HOME` is what
`Motion_HomePosition` runs with, `BHAND_JOINT_PD` is what `SetMotionType`
installs for joint PD (same stiffness, more damping).

The compliant profiles are derived from `BHAND_HOME` by `scaled()`, which
multiplies stiffness by `s` and damping by `sqrt(s)`. That keeps the damping
ratio constant, so a softer hand stays just as well damped instead of turning
springy — the usual mistake when someone scales kp and kd together.

    from allegro_hand_v5 import AllegroHand, COMPLIANT

    with AllegroHand("can0", gains=COMPLIANT) as hand:
        ...
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Union

import numpy as np

from allegro_hand_v5.protocol import (
    FINGER_NAMES,
    JOINT_NAMES,
    NUM_JOINTS,
    NUM_JOINTS_PER_FINGER,
)


def _per_finger(fingers: Sequence[float], thumb: Sequence[float]) -> np.ndarray:
    """Build a 16-vector from 4 values for index/middle/ring and 4 for the thumb."""
    if len(fingers) != NUM_JOINTS_PER_FINGER or len(thumb) != NUM_JOINTS_PER_FINGER:
        raise ValueError("expected 4 values per finger")
    return np.array(list(fingers) * 3 + list(thumb), dtype=np.float64)


@dataclass(frozen=True)
class GainProfile:
    """
    Per-joint PD gains and torque clamp.

    Attributes:
        kp: (16,) stiffness, Nm/rad.
        kd: (16,) damping, Nm/(rad/s).
        max_torque: (16,) per-joint torque clamp, Nm.
    """

    name: str
    kp: np.ndarray
    kd: np.ndarray
    max_torque: np.ndarray

    def __post_init__(self):
        for attr in ("kp", "kd", "max_torque"):
            value = np.asarray(getattr(self, attr), dtype=np.float64)
            if value.size == 1:
                value = np.full(NUM_JOINTS, float(value))
            if value.shape != (NUM_JOINTS,):
                raise ValueError(f"{attr} must be scalar or length {NUM_JOINTS}, got {value.shape}")
            object.__setattr__(self, attr, value)

    def scaled(self, stiffness: float, name: str | None = None) -> "GainProfile":
        """
        Softer or stiffer version of this profile, at the same damping ratio.

        `kp` scales by `stiffness` and `kd` by `sqrt(stiffness)`, because the
        damping ratio of a second-order joint goes as `kd / sqrt(kp)`. Scaling
        both by the same factor would leave the hand underdamped.

        Args:
            stiffness: Multiplier on kp. 0.5 halves the stiffness.
            name: Name for the result; defaults to "<name>x<stiffness>".
        """
        if stiffness <= 0:
            raise ValueError("stiffness multiplier must be positive")
        return GainProfile(
            name=name or f"{self.name}x{stiffness:g}",
            kp=self.kp * stiffness,
            kd=self.kd * math.sqrt(stiffness),
            max_torque=self.max_torque.copy(),
        )

    def with_max_torque(self, max_torque: Union[float, Sequence[float]]) -> "GainProfile":
        """Copy with a different torque clamp."""
        return GainProfile(name=self.name, kp=self.kp.copy(), kd=self.kd.copy(),
                           max_torque=max_torque)

    def replace(self, kp=None, kd=None, max_torque=None, name=None) -> "GainProfile":
        """Copy with individual fields overridden."""
        return GainProfile(
            name=name or self.name,
            kp=self.kp.copy() if kp is None else kp,
            kd=self.kd.copy() if kd is None else kd,
            max_torque=self.max_torque.copy() if max_torque is None else max_torque,
        )

    @property
    def damping_ratio_index(self) -> np.ndarray:
        """`kd / sqrt(kp)` per joint — constant across `scaled()` variants."""
        return self.kd / np.sqrt(self.kp)

    def table(self) -> str:
        """Formatted per-finger view, in the layout WONIK's tools print."""
        lines = [f"GainProfile {self.name!r}"]
        for f, finger in enumerate(FINGER_NAMES):
            s = slice(f * NUM_JOINTS_PER_FINGER, (f + 1) * NUM_JOINTS_PER_FINGER)
            kp = "  ".join(f"{v:6.3f}" for v in self.kp[s])
            kd = "  ".join(f"{v:6.3f}" for v in self.kd[s])
            lines.append(f"  {finger:<7} kp: {kp}   kd: {kd}")
        lo, hi = self.max_torque.min(), self.max_torque.max()
        clamp = f"{lo:.3f}" if lo == hi else f"{lo:.3f}..{hi:.3f}"
        lines.append(f"  max torque: {clamp} Nm")
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.table()


#: Default per-joint torque clamp, Nm. 0.15 Nm is 215 mA, inside the 240 mA
#: limit the reference stack uses.
DEFAULT_MAX_TORQUE = 0.1


#: Gains libBHand runs during Motion_HomePosition. Note the MCP joints (index 1
#: of each finger) carry ~3x the damping of the others.
BHAND_HOME = GainProfile(
    name="bhand_home",
    kp=_per_finger([1.0, 1.0, 1.0, 1.0], [0.8, 0.8, 0.8, 0.8]),
    kd=_per_finger([0.03, 0.10, 0.03, 0.03], [0.03, 0.03, 0.03, 0.03]),
    max_torque=DEFAULT_MAX_TORQUE,
)

#: Gains libBHand installs for joint PD (SetMotionType resets to these).
#: Same stiffness as BHAND_HOME, noticeably more damping on the fingers.
BHAND_JOINT_PD = GainProfile(
    name="bhand_joint_pd",
    kp=_per_finger([1.0, 1.0, 1.0, 1.0], [0.8, 0.8, 0.8, 0.8]),
    kd=_per_finger([0.04, 0.15, 0.04, 0.04], [0.03, 0.03, 0.03, 0.03]),
    max_torque=DEFAULT_MAX_TORQUE,
)

#: Half the stiffness of BHAND_HOME, same damping ratio. Backdrivable enough to
#: push the fingers around by hand while still holding a pose.
COMPLIANT = BHAND_HOME.scaled(0.5, name="compliant")

#: Quarter stiffness. Very backdrivable; expect visible sag under gravity,
#: since there is no gravity compensation.
SOFT = BHAND_HOME.scaled(0.25, name="soft")

#: Minimal stiffness with a low clamp, for first power-on and for testing a
#: hand you do not trust yet.
SAFE = BHAND_HOME.scaled(0.25, name="safe").with_max_torque(0.05)

#: Zero gains: the PD produces nothing, so the hand hangs limp in position mode.
ZERO = GainProfile(name="zero", kp=0.0, kd=0.0, max_torque=DEFAULT_MAX_TORQUE)

#: Default when none is given.
DEFAULT = BHAND_HOME

PROFILES = {
    p.name: p
    for p in (BHAND_HOME, BHAND_JOINT_PD, COMPLIANT, SOFT, SAFE, ZERO)
}


def get_profile(name: str) -> GainProfile:
    """Look a profile up by name. Raises KeyError listing the valid names."""
    try:
        return PROFILES[name]
    except KeyError:
        raise KeyError(f"unknown gain profile {name!r}; choose from {sorted(PROFILES)}") from None


__all__ = [
    "GainProfile",
    "BHAND_HOME",
    "BHAND_JOINT_PD",
    "COMPLIANT",
    "SOFT",
    "SAFE",
    "ZERO",
    "DEFAULT",
    "PROFILES",
    "get_profile",
    "DEFAULT_MAX_TORQUE",
    "JOINT_NAMES",
]
