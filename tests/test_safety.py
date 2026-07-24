"""Tests for the emergency torque-off sequence and shutdown idempotency."""

import struct

from allegro_hand_v5 import AllegroHand
from allegro_hand_v5 import constants as C
from allegro_hand_v5.driver import emergency_off_frames


def test_emergency_off_frames_zero_then_servo_off():
    frames = emergency_off_frames()
    # 4 zero-current set-torque frames (one per finger) + one SERVO_OFF
    assert len(frames) == C.NUM_FINGERS + 1

    for f in range(C.NUM_FINGERS):
        frame = frames[f]
        assert frame.msg_id == C.ID_SET_TORQUE + f
        assert struct.unpack("<4h", frame.data) == (0, 0, 0, 0)  # zero current

    servo_off = frames[-1]
    assert servo_off.msg_id == C.ID_SYSTEM_OFF
    assert servo_off.data == b""


def test_disconnect_is_idempotent_without_connect():
    hand = AllegroHand(channel="can0")  # never connected
    hand.disconnect()
    hand.disconnect()  # must not raise
    assert hand._running.value == 0
