"""Wire format: every encoder and decoder in `protocol`."""

import math
import struct

import numpy as np
import pytest

from allegro_hand_v5 import protocol as proto
from allegro_hand_v5.protocol import ErrorFlag, HandInfo, MsgID


def test_joint_tables_are_consistent():
    assert proto.NUM_JOINTS == 16
    assert len(proto.JOINT_NAMES) == proto.NUM_JOINTS
    assert len(set(proto.JOINT_NAMES)) == proto.NUM_JOINTS
    assert proto.JOINT_INDEX["thumb_ip"] == 15
    assert proto.PLUS_GEARED_JOINTS == (1, 5, 9)
    for i in proto.PLUS_GEARED_JOINTS:
        assert proto.JOINT_NAMES[i].endswith("mcp2")


def test_frame_id_shifts_by_two():
    assert proto.frame_id(MsgID.SERVO_ON) == 0x040 << 2
    for msg_id in MsgID:
        assert proto.message_id(proto.frame_id(msg_id)) == msg_id


def test_finger_slice_partitions_the_hand():
    covered = np.zeros(16, dtype=int)
    for finger in range(4):
        covered[proto.finger_slice(finger)] += 1
    assert (covered == 1).all()


def test_position_scale_is_the_manual_value():
    assert proto.POSITION_SCALE == pytest.approx(math.radians(0.088))
    # 0.088 deg per LSB: 1000 counts is 88 degrees
    assert math.degrees(1000 * proto.POSITION_SCALE) == pytest.approx(88.0)


# ==================== Encoding ====================


def test_encode_currents_packs_four_int16_little_endian():
    arb_id, data = proto.encode_currents(2, [1, -2, 3, -4])
    assert arb_id == proto.frame_id(MsgID.SET_TORQUE + 2)
    assert struct.unpack("<4h", data) == (1, -2, 3, -4)


def test_encode_currents_saturates_at_the_hardware_limit():
    _, data = proto.encode_currents(0, [1e6, -1e6, 240, -240])
    assert struct.unpack("<4h", data) == (240, -240, 240, -240)


def test_encode_currents_rounds_rather_than_truncates():
    _, data = proto.encode_currents(0, [1.6, -1.6, 0.4, -0.4])
    assert struct.unpack("<4h", data) == (2, -2, 0, 0)


@pytest.mark.parametrize("finger", [-1, 4])
def test_encode_currents_rejects_bad_finger(finger):
    with pytest.raises(ValueError):
        proto.encode_currents(finger, [0, 0, 0, 0])


def test_encode_currents_rejects_wrong_length():
    with pytest.raises(ValueError):
        proto.encode_currents(0, [0, 0, 0])


def test_encode_pose_uses_the_position_scale():
    arb_id, data = proto.encode_pose(1, [proto.POSITION_SCALE * 100, 0, 0, 0])
    assert arb_id == proto.frame_id(MsgID.SET_POSE + 1)
    assert struct.unpack("<4h", data)[0] == 100


def test_encode_period_is_three_int16():
    arb_id, data = proto.encode_period(3, 0, 10)
    assert arb_id == proto.frame_id(MsgID.SET_PERIOD)
    assert len(data) == 6
    assert struct.unpack("<3h", data) == (3, 0, 10)


# ==================== Decoding ====================


def test_decode_position_roundtrips_encoder_counts():
    data = struct.pack("<4h", 0, 100, -100, 1000)
    q = proto.decode_position(data)
    assert q == pytest.approx(np.array([0, 100, -100, 1000]) * proto.POSITION_SCALE)


def test_decode_position_needs_eight_bytes():
    with pytest.raises(ValueError):
        proto.decode_position(b"\x00\x00")


def test_decode_pressure_returns_both_fingers():
    assert proto.decode_pressure(struct.pack("<2i", 120, 4000)) == (120, 4000)


@pytest.mark.parametrize("raw", [-1, -100000, proto.PRESSURE_VALID_MAX + 1])
def test_decode_pressure_zeroes_implausible_readings(raw):
    assert proto.decode_pressure(struct.pack("<2i", raw, raw)) == (0, 0)


def test_pressure_ids_map_to_the_right_fingers():
    assert proto.PRESSURE_ID_TO_FINGER[MsgID.PRESSURE_1] == 0
    assert proto.PRESSURE_ID_TO_FINGER[MsgID.PRESSURE_2] == 2
    # The manual's IDs and the ones current firmware actually uses both work.
    assert proto.PRESSURE_ID_TO_FINGER[MsgID.PRESSURE_1_LEGACY] == 0
    assert proto.PRESSURE_ID_TO_FINGER[MsgID.PRESSURE_2_LEGACY] == 2


def test_decode_temperature_is_one_byte_per_joint():
    assert list(proto.decode_temperature(bytes([30, 31, 32, 33]))) == [30, 31, 32, 33]


def test_decode_imu_is_three_int16():
    assert list(proto.decode_imu(struct.pack("<3h", -1, 2, -3))) == [-1, 2, -3]


def test_decode_info_reads_two_versions():
    assert proto.decode_info(struct.pack("<2H", 0x0500, 0x0101)) == (0x0500, 0x0101)


def test_decode_serial_strips_padding():
    assert proto.decode_serial(b"5TBR0017") == "5TBR0017"
    assert proto.decode_serial(b"5TBR00\x00\x00") == "5TBR00"


def test_decode_error_reads_motor_and_code():
    err = proto.decode_error(bytes([5, int(ErrorFlag.OVERLOAD | ErrorFlag.OVERHEATING)]))
    assert err.motor_id == 5
    assert err.code == ErrorFlag.OVERLOAD | ErrorFlag.OVERHEATING
    assert err.joint_name == "middle_mcp2"
    assert "overload" in str(err) and "overheating" in str(err)


def test_error_flag_describes_nothing_and_reserved_bits():
    assert ErrorFlag(0).describe() == "none"
    assert "reserved" in ErrorFlag(0b10).describe()


def test_joint_error_names_unknown_motors():
    assert proto.JointError(99, ErrorFlag.NONE).joint_name == "motor_99"


# ==================== HandInfo ====================


@pytest.mark.parametrize("serial,handedness,hw_type", [
    ("5TBR0017", "right", "B"),
    ("5TAL0001", "left", "A"),
    ("", "unknown", "unknown"),
    ("5TXX0001", "unknown", "unknown"),
])
def test_hand_info_reads_the_serial_number(serial, handedness, hw_type):
    info = HandInfo(serial_number=serial)
    assert info.handedness == handedness
    assert info.hardware_type == hw_type
    assert info.complete is bool(serial)


def test_hand_info_is_complete_without_a_version():
    # Real firmware often ignores the Information RTR; the serial is enough.
    assert HandInfo(serial_number="5TBR0017").complete
    assert "?" in str(HandInfo(serial_number="5TBR0017"))
