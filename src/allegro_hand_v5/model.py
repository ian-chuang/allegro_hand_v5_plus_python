"""Hand model metadata: handedness/type and per-configuration joint limits.

Joint limits come from :data:`allegro_hand_v5.constants.JOINT_LIMITS`, keyed by
``"<handedness>_<type>"`` (e.g. ``"right_B"``). No URDF is required.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import constants as C
from .protocol import HandSerial


@dataclass(frozen=True)
class HandModel:
    """Static description of a specific hand, resolved from its serial number."""

    serial: str
    is_right: bool
    is_type_a: bool

    @property
    def handedness(self) -> str:
        return "right" if self.is_right else "left"

    @property
    def hand_type(self) -> str:
        return "A" if self.is_type_a else "B"

    @property
    def is_plus(self) -> bool:
        """Type B hands (e.g. the Plus) need MCP-2 torque halving on fingers 0/1/2."""
        return not self.is_type_a

    @property
    def config_key(self) -> str:
        """Key into ``constants.JOINT_LIMITS``, e.g. ``"right_B"``."""
        return f"{self.handedness}_{self.hand_type}"

    @property
    def joint_limits_lower(self) -> np.ndarray:
        return np.array(C.JOINT_LIMITS[self.config_key][0], dtype=np.float64)

    @property
    def joint_limits_upper(self) -> np.ndarray:
        return np.array(C.JOINT_LIMITS[self.config_key][1], dtype=np.float64)

    def clamp_positions(self, q) -> np.ndarray:
        """Clamp a 16-vector of joint angles [rad] into the model's joint limits."""
        return np.clip(np.asarray(q, dtype=np.float64),
                       self.joint_limits_lower, self.joint_limits_upper)

    @classmethod
    def from_serial(cls, hs: HandSerial) -> "HandModel":
        return cls(serial=hs.serial, is_right=hs.is_right, is_type_a=hs.is_type_a)

    @classmethod
    def default(cls, is_right: bool = True, is_type_a: bool = False) -> "HandModel":
        """Fallback model when the serial has not been read yet. Defaults to right / B."""
        return cls(serial="UNKNOWN", is_right=is_right, is_type_a=is_type_a)
