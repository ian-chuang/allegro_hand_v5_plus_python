"""Unit tests for the pure protocol codec — no hardware required."""

import math
import struct

import pytest

from allegro_hand_v5 import constants as C
from allegro_hand_v5 import protocol as P


def test_arbitration_id_roundtrip():
    for msg_id in (0x40, 0x60, 0x20, 0x88, 0xEE):
        assert P.from_arbitration_id(P.to_arbitration_id(msg_id)) == msg_id
    assert P.to_arbitration_id(0x60) == 0x180  # 0x60 << 2


def test_servo_frames_have_no_data():
    on, off = P.encode_servo_on(), P.encode_servo_off()
    assert on.msg_id == C.ID_SYSTEM_ON and on.data == b""
    assert off.msg_id == C.ID_SYSTEM_OFF and off.data == b""


def test_encode_set_torque_layout():
    f = P.encode_set_torque(2, [100, -200, 0, 240])
    assert f.msg_id == C.ID_SET_TORQUE + 2
    assert struct.unpack("<4h", f.data) == (100, -200, 0, 240)


def test_encode_set_torque_validates():
    with pytest.raises(ValueError):
        P.encode_set_torque(4, [0, 0, 0, 0])
    with pytest.raises(ValueError):
        P.encode_set_torque(0, [0, 0, 0])


def test_set_period_and_stop():
    f = P.encode_set_period((3, 0, 0))
    assert struct.unpack("<3h", f.data) == (3, 0, 0)
    assert struct.unpack("<3h", P.encode_stop_streams().data) == (0, 0, 0)


def test_rtr_requests():
    assert P.request_hand_info().is_rtr
    assert P.request_serial().is_rtr
    assert P.request_finger_pose(3).msg_id == C.ID_RTR_FINGER_POSE + 3


def test_torque_nm_to_ma_scale_and_clamp():
    tau = [0.0] * C.DOF
    tau[0] = 0.1  # 0.1 * 1430 = 143 mA
    ma = P.torque_nm_to_ma(tau, is_plus=False)
    assert ma[0] == pytest.approx(143.0)
    # clamp
    tau[4] = 10.0
    ma = P.torque_nm_to_ma(tau, is_plus=False)
    assert ma[4] == C.TORQUE_LIMIT_MA


def test_torque_plus_halves_mcp2():
    tau = [0.0] * C.DOF
    for j in C.PLUS_HALVED_JOINTS:
        tau[j] = 0.1
    ma_std = P.torque_nm_to_ma(tau, is_plus=False)
    ma_plus = P.torque_nm_to_ma(tau, is_plus=True)
    for j in C.PLUS_HALVED_JOINTS:
        assert ma_plus[j] == pytest.approx(0.5 * ma_std[j])
    # thumb MCP-equivalent (index 13) is never halved
    assert 13 not in C.PLUS_HALVED_JOINTS


def test_decode_finger_pose():
    raw = (1000, -1000, 0, 2000)
    angles = P.decode_finger_pose(0, struct.pack("<4h", *raw))
    assert angles[0] == pytest.approx(1000 * math.radians(0.088))
    assert angles[1] == pytest.approx(-1000 * math.radians(0.088))
    assert angles[2] == 0.0


def test_decode_fingertip_groups():
    data = struct.pack("<2i", 111, 222)
    assert P.decode_fingertip(C.ID_FINGERTIP_0, data) == (0, 111, 1, 222)
    assert P.decode_fingertip(C.ID_FINGERTIP_2, data) == (2, 111, 3, 222)


def test_decode_serial_handedness_and_type():
    # bytes: [_, _, type, hand, ...]; type 'A', hand 'R'
    data = bytes([ord("0"), ord("0"), ord("A"), ord("R"), 0, 0, 0, 0])
    hs = P.decode_serial(data)
    assert hs.is_right and hs.is_type_a
    assert hs.handedness == "right" and hs.hand_type == "A"

    data_left_b = bytes([ord("0"), ord("0"), ord("B"), ord("L"), 0, 0, 0, 0])
    hs2 = P.decode_serial(data_left_b)
    assert not hs2.is_right and not hs2.is_type_a


def test_decode_hand_info():
    data = struct.pack("<HH", 0x0102, 0x0304) + bytes([0, 0, 0x01])
    info = P.decode_hand_info(data)
    assert info.hardware_version == 0x0102
    assert info.firmware_version == 0x0304
    assert info.servo_on is True


def test_decode_error_flags():
    err = P.decode_error(bytes([5, (1 << 5) | (1 << 0)]))
    assert err.motor_id == 5
    assert set(err.flags) == {"overload", "input_voltage"}
