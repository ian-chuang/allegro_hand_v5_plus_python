"""Python driver and control library for the Allegro Hand V5 (F4) / (F4) Plus.

The V5 hardware is torque(current)-only over CAN, so position control is closed on the
host. :class:`AllegroHand` runs that control loop in a separate process (its own GIL) for
reliable real-time timing, communicating with the parent through shared memory.

Modules:
  * :mod:`allegro_hand_v5.protocol`  - pure CAN frame codec (encode/decode)
  * :mod:`allegro_hand_v5.driver`    - AllegroHand: process-based real-time driver
  * :mod:`allegro_hand_v5.config`    - DriverConfig: command mode, limits, PID gains
  * :mod:`allegro_hand_v5.model`     - handedness/type + per-config joint limits
  * :mod:`allegro_hand_v5.constants` - verified hardware constants
"""

from __future__ import annotations

from . import constants, protocol
from .config import DriverConfig
from .driver import AllegroHand, pd_torque
from .model import HandModel
from .protocol import HandInfo, HandSerial, MotorError

__all__ = [
    "AllegroHand",
    "DriverConfig",
    "HandModel",
    "HandInfo",
    "HandSerial",
    "MotorError",
    "pd_torque",
    "constants",
    "protocol",
]

__version__ = "0.1.0"
