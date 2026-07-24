"""Driver configuration.

Bundles everything tunable: the torque→current scaling, per-joint current clamps, "Plus"
handling, and the host-side PID gains.  Construct one and pass it to
:class:`~allegro_hand_v5.driver.AllegroHand`.

The **V5 hardware is current-controlled**: the value on the wire is a motor-current
setpoint in milliamps (mA), and the mainboard closes a current loop per joint.  Desired
joint torque [Nm] is converted to current via :attr:`DriverConfig.nm_to_ma` (≈1.43e3 from
Wonik's driver) and clamped per joint by :attr:`DriverConfig.max_current_ma`.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import constants as C


def _as_dof_vector(value, name: str) -> np.ndarray:
    """Broadcast a scalar or length-16 iterable to a float64 (16,) array."""
    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim == 0:
        arr = np.full(C.DOF, float(arr))
    if arr.shape != (C.DOF,):
        raise ValueError(f"{name} must be a scalar or length-{C.DOF}, got shape {arr.shape}")
    return arr.copy()


@dataclass
class DriverConfig:
    """All tunable driver + PD control parameters. Sensible, conservative defaults."""

    # -- motor current command ---------------------------------------------
    #: Desired joint torque [Nm] → motor current [mA]. ≈1.43e3 from Wonik's driver.
    nm_to_ma: float = C.NM_TO_MA

    #: Per-joint absolute clamp on the current command [mA]. (16,) or scalar.
    #: Defaults to the firmware's ±240 mA hard limit — treat that as the ceiling.
    max_current_ma: np.ndarray = field(
        default_factory=lambda: np.full(C.DOF, float(C.TORQUE_LIMIT_MA))
    )

    #: Joints whose current is halved on "Plus" (type B) hands (2× gear ratio). Set to
    #: an empty tuple to disable. Only applied when the connected hand reports type B.
    plus_halved_joints: tuple = C.PLUS_HALVED_JOINTS

    # -- host-side PID position controller ---------------------------------
    kp: np.ndarray = field(default_factory=lambda: np.array([
        0.6, 0.9, 0.9, 0.6,
        0.6, 0.9, 0.9, 0.6,
        0.6, 0.9, 0.9, 0.6,
        1.0, 0.6, 0.9, 0.9,
    ]))
    kd: np.ndarray = field(default_factory=lambda: np.full(C.DOF, 0.02))
    ki: np.ndarray = field(default_factory=lambda: np.zeros(C.DOF))
    rate_hz: float = C.DEFAULT_CONTROL_HZ
    i_clamp: float = 0.05
    #: Clamp position targets into the model's joint limits before control.
    clamp_to_limits: bool = True

    # ----------------------------------------------------------------------
    def __post_init__(self):
        self.max_current_ma = np.abs(_as_dof_vector(self.max_current_ma, "max_current_ma"))
        self.kp = _as_dof_vector(self.kp, "kp")
        self.kd = _as_dof_vector(self.kd, "kd")
        self.ki = _as_dof_vector(self.ki, "ki")

    # -- limit helpers ------------------------------------------------------
    def set_max_torque(self, max_torque_nm) -> "DriverConfig":
        """Set the per-joint current clamp from a torque limit [Nm] (scalar or (16,)).

        ``max_current_ma = |max_torque_nm| * nm_to_ma``. Returns self for chaining.
        """
        tau = _as_dof_vector(max_torque_nm, "max_torque_nm")
        self.max_current_ma = np.abs(tau * self.nm_to_ma)
        return self

    def set_max_current(self, max_current_ma) -> "DriverConfig":
        """Set the per-joint clamp directly in mA."""
        self.max_current_ma = np.abs(_as_dof_vector(max_current_ma, "max_current_ma"))
        return self

    def set_joint_max_current(self, joint: int, value_ma: float) -> "DriverConfig":
        self.max_current_ma[joint] = abs(float(value_ma))
        return self

    @property
    def max_torque_nm(self) -> np.ndarray:
        """The effective per-joint torque ceiling implied by ``max_current_ma`` [Nm]."""
        return self.max_current_ma / self.nm_to_ma

    # -- conversion ---------------------------------------------------------
    def clamp_current(self, current_ma) -> np.ndarray:
        v = _as_dof_vector(current_ma, "current_ma")
        return np.clip(v, -self.max_current_ma, self.max_current_ma)

    def torque_to_current(self, torque_nm, is_plus: bool = False) -> np.ndarray:
        """Convert a 16-vector of joint torques [Nm] to clamped motor currents [mA].

        Applies ``nm_to_ma``, optional Plus halving, then the per-joint clamp.
        """
        vals = _as_dof_vector(torque_nm, "torque_nm") * self.nm_to_ma
        if is_plus and self.plus_halved_joints:
            vals[list(self.plus_halved_joints)] *= 0.5
        return self.clamp_current(vals)

    # -- named presets ------------------------------------------------------
    @classmethod
    def safe(cls, max_torque_nm: float = 0.02, kp_scale: float = 0.5) -> "DriverConfig":
        """A deliberately gentle config for first-run / bench testing.

        Tiny per-joint torque ceiling (default 0.02 Nm ≈ 29 mA, well under the 0.23 Nm
        nominal) and softened P gains, so a runaway command can't drive the joints hard.
        """
        cfg = cls()
        cfg.kp = cfg.kp * kp_scale
        cfg.set_max_torque(max_torque_nm)
        return cfg
