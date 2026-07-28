"""
PD gains for the host-side position controller.

The hand takes one actuation command: a motor current per joint. So the PD is
written directly in current, with no torque model in between:

    current[mA] = kp * (q_desired - q) - kd * dq        clipped to +/- max_current

`kp` is mA per radian of error, `kd` is mA per rad/s, `max_current` is mA. There
is nothing else in the loop — see `driver._control_loop`.

The presets below are the gains WONIK's `libBHand.so` uses for its home motion
(kp = 1.0 Nm/rad on the fingers, 0.8 on the thumb; see `docs/bhand_gains.md`),
converted at that stack's 1.43 A/Nm and rounded. The MCP-2 joints of index,
middle and ring (indices 1, 5, 9) carry **half** the numbers of their
neighbours: a "Plus" (type B) hand gears them 2:1, so half the current there is
the same joint torque. On a non-geared type A hand, double those three entries.

    from allegro_hand_v5 import AllegroHand, COMPLIANT

    with AllegroHand("can0", gains=COMPLIANT) as hand:
        ...
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence, Union

import numpy as np

from allegro_hand_v5.protocol import (
    FINGER_NAMES,
    NUM_JOINTS,
    NUM_JOINTS_PER_FINGER,
    MAX_CURRENT_MA,
)

Vector = Union[float, Sequence[float], np.ndarray]


def _hand(finger: Sequence[float], thumb: Sequence[float]) -> np.ndarray:
    """16-vector from 4 values for index/middle/ring and 4 for the thumb."""
    if len(finger) != NUM_JOINTS_PER_FINGER or len(thumb) != NUM_JOINTS_PER_FINGER:
        raise ValueError("expected 4 values per finger")
    return np.array(list(finger) * 3 + list(thumb), dtype=np.float64)


@dataclass(frozen=True)
class Gains:
    """
    Per-joint PD gains and current limit. Scalars broadcast to all 16 joints.

    Attributes:
        kp: (16,) stiffness, mA per rad of position error.
        kd: (16,) damping, mA per rad/s.
        max_current: (16,) per-joint current limit, mA. Also capped by the
            hardware limit `protocol.MAX_CURRENT_MA` (240 mA) on the wire.
    """

    kp: np.ndarray
    kd: np.ndarray
    max_current: np.ndarray
    name: str = "custom"

    def __post_init__(self):
        for attr in ("kp", "kd", "max_current"):
            value = np.asarray(getattr(self, attr), dtype=np.float64)
            if value.ndim == 0:
                value = np.full(NUM_JOINTS, float(value))
            if value.shape != (NUM_JOINTS,):
                raise ValueError(
                    f"{attr} must be a scalar or {NUM_JOINTS} values, got {value.shape}"
                )
            if np.any(value < 0):
                raise ValueError(f"{attr} must not be negative")
            object.__setattr__(self, attr, value)

    def replace(self, kp: Vector = None, kd: Vector = None,
                max_current: Vector = None, name: str = None) -> "Gains":
        """Copy with individual fields overridden."""
        return Gains(
            kp=self.kp if kp is None else kp,
            kd=self.kd if kd is None else kd,
            max_current=self.max_current if max_current is None else max_current,
            name=name or self.name,
        )

    def __str__(self) -> str:
        lines = [f"Gains {self.name!r}  (mA/rad, mA per rad/s, mA)"]
        for f, finger in enumerate(FINGER_NAMES):
            s = slice(f * NUM_JOINTS_PER_FINGER, (f + 1) * NUM_JOINTS_PER_FINGER)
            lines.append(
                f"  {finger:<7}"
                f" kp {' '.join(f'{v:6.0f}' for v in self.kp[s])}  "
                f" kd {' '.join(f'{v:5.0f}' for v in self.kd[s])}  "
                f" max {' '.join(f'{v:4.0f}' for v in self.max_current[s])}"
            )
        return "\n".join(lines)


#: Vendor-equivalent stiffness. What `libBHand`'s home motion runs with.
DEFAULT = Gains(
    name="default",
    kp=_hand([1400, 700, 1400, 1400], [1150, 1150, 1150, 1150]),
    kd=_hand([45, 70, 45, 45], [45, 45, 45, 45]),
    max_current=_hand([150, 75, 150, 150], [150, 150, 150, 150]),
)

#: Half the stiffness of DEFAULT, damping scaled by sqrt(0.5) so the joints stay
#: as well damped. Backdrivable by hand while still holding a pose.
COMPLIANT = Gains(
    name="compliant",
    kp=_hand([700, 350, 700, 700], [575, 575, 575, 575]),
    kd=_hand([32, 50, 32, 32], [32, 32, 32, 32]),
    max_current=DEFAULT.max_current,
)

#: Quarter stiffness. Very backdrivable; expect visible sag under gravity, since
#: there is no gravity compensation anywhere in this package.
SOFT = Gains(
    name="soft",
    kp=_hand([350, 175, 350, 350], [290, 290, 290, 290]),
    kd=_hand([22, 35, 22, 22], [22, 22, 22, 22]),
    max_current=DEFAULT.max_current,
)

#: Soft, with a low current limit. For first power-on and for a hand you do not
#: trust yet: a mistake costs you a weak push rather than a stalled motor.
SAFE = SOFT.replace(name="safe", max_current=_hand([50, 25, 50, 50], [50, 50, 50, 50]))

#: No feedback at all: position mode commands zero current, so the hand hangs limp.
ZERO = Gains(name="zero", kp=0.0, kd=0.0, max_current=MAX_CURRENT_MA)

PRESETS: Dict[str, Gains] = {g.name: g for g in (DEFAULT, COMPLIANT, SOFT, SAFE, ZERO)}


def preset(name: str) -> Gains:
    """Look a preset up by name. Raises KeyError listing the valid names."""
    try:
        return PRESETS[name]
    except KeyError:
        raise KeyError(f"unknown gain preset {name!r}; choose from {sorted(PRESETS)}") from None
