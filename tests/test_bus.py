"""`AllegroCANBus` against a simulated hand."""

import struct
import time

import numpy as np
import pytest

from allegro_hand_v5 import AllegroCANBus, protocol as proto
from allegro_hand_v5.exceptions import AllegroCANError, AllegroConnectionError, AllegroTimeoutError
from allegro_hand_v5.protocol import ErrorFlag, MsgID
from fake_hand import FakeHand


def drain(bus, seconds=0.05):
    """Poll for a while, so the simulated hand's periodic reports arrive."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bus.poll(timeout=0.002)


# ==================== Lifecycle ====================


def test_open_is_idempotent_and_close_shuts_the_socket_down(fake_hand):
    bus = AllegroCANBus("test0", bus_factory=lambda: fake_hand)
    bus.open()
    bus.open()
    assert bus.is_open
    bus.close()
    assert not bus.is_open
    assert fake_hand.closed
    bus.close()  # no error the second time


def test_operations_before_open_raise():
    bus = AllegroCANBus("test0", bus_factory=lambda: FakeHand())
    with pytest.raises(AllegroCANError):
        bus.servo_on()


def test_a_failing_factory_becomes_a_connection_error():
    def boom():
        raise OSError("no such device")

    with pytest.raises(AllegroConnectionError, match="test0"):
        AllegroCANBus("test0", bus_factory=boom).open()


def test_close_leaves_the_hand_disarmed(bus, fake_hand):
    bus.servo_on()
    bus.close()
    assert fake_hand.servo_on is False
    assert (fake_hand.currents == 0).all()


# ==================== Transmit ====================


def test_servo_commands_track_the_hand(bus, fake_hand):
    assert bus.servo_is_on is False
    bus.servo_on()
    assert bus.servo_is_on and fake_hand.servo_on
    bus.servo_off()
    assert not bus.servo_is_on and not fake_hand.servo_on


def test_send_currents_writes_one_frame_per_finger(bus, fake_hand):
    fake_hand.sent.clear()
    bus.send_currents(np.arange(16, dtype=float))
    assert fake_hand.sent_ids() == [MsgID.SET_TORQUE + f for f in range(4)]
    assert fake_hand.currents == pytest.approx(np.arange(16))


def test_send_currents_saturates_at_the_hardware_limit(bus, fake_hand):
    bus.send_currents(np.full(16, 5000.0))
    assert (fake_hand.currents == proto.MAX_CURRENT_MA).all()


def test_send_currents_rejects_the_wrong_length(bus):
    with pytest.raises(ValueError):
        bus.send_currents(np.zeros(4))


def test_send_pose_uses_the_pose_ids(bus, fake_hand):
    fake_hand.sent.clear()
    bus.send_pose(np.zeros(16))
    assert fake_hand.sent_ids() == [MsgID.SET_POSE + f for f in range(4)]


def test_set_period_reaches_the_hand(bus, fake_hand):
    bus.set_period(5, 20, 100)
    assert (fake_hand.position_period_ms, fake_hand.imu_period_ms,
            fake_hand.temperature_period_ms) == (5, 20, 100)


def test_pick_place_and_calibration_are_single_frames(bus, fake_hand):
    fake_hand.sent.clear()
    bus.pick()
    bus.place()
    bus.start_motor_calibration()
    assert fake_hand.sent_ids() == [MsgID.PICK, MsgID.PLACE, MsgID.CALIBRATE]


def test_a_full_transmit_queue_is_counted_not_raised(bus, fake_hand, monkeypatch):
    import can

    def enobufs(msg, timeout=None):
        raise can.CanError("No buffer space available")

    monkeypatch.setattr(fake_hand, "send", enobufs)
    bus.send_currents(np.zeros(16))  # must not raise: the loop has to keep running
    assert bus.state.tx_dropped == 4


def test_other_can_errors_do_raise(bus, fake_hand, monkeypatch):
    import can

    monkeypatch.setattr(fake_hand, "send", lambda *a, **k: (_ for _ in ()).throw(
        can.CanError("bus is on fire")))
    with pytest.raises(AllegroCANError):
        bus.servo_on()


# ==================== Receive ====================


def test_handshake_identifies_the_hand_and_starts_the_stream(bus, fake_hand):
    info = bus.handshake(timeout=1.0, position_period_ms=3)
    assert info.serial_number == "5TBR0017"
    assert info.handedness == "right"
    assert info.hardware_type == "B"
    assert info.hardware_version == 0x0500
    assert fake_hand.position_period_ms == 3
    assert bus.servo_is_on, "the hand only reports positions with the servos engaged"
    assert bus.state.positions_fresh


def test_handshake_works_when_the_hand_ignores_the_info_request(fake_hand):
    fake_hand.answer_info = False
    bus = AllegroCANBus("test0", bus_factory=lambda: fake_hand)
    bus.open()
    info = bus.handshake(timeout=1.0)
    assert info.complete, "the serial number alone is enough to identify the hand"
    assert info.hardware_version is None
    bus.close()


def test_positions_decode_into_state(bus, fake_hand):
    fake_hand.positions[:] = np.linspace(-0.5, 0.5, 16)
    bus.handshake(timeout=1.0)
    assert bus.state.positions == pytest.approx(fake_hand.positions, abs=proto.POSITION_SCALE)


def test_position_flags_track_which_fingers_reported(bus, fake_hand):
    bus.handshake(timeout=1.0)
    assert bus.state.positions_fresh
    bus.state.clear_position_flags()
    assert not bus.state.positions_fresh
    assert bus.state.position_flags == 0
    drain(bus)
    assert bus.state.positions_fresh


def test_wait_for_positions_times_out_when_the_hand_is_silent(bus, fake_hand):
    fake_hand.position_period_ms = 0
    with pytest.raises(AllegroTimeoutError, match="0000"):
        bus.wait_for_positions(timeout=0.05)


def test_pressures_decode_for_both_id_pairs(bus, fake_hand):
    bus.handshake(timeout=1.0)
    assert list(bus.state.pressures) == [10, 20, 30, 40]

    # The v1.3 manual documents 0x50/0x52; firmware uses 0xF0/0xF2. Both decode.
    fake_hand._queue(MsgID.PRESSURE_1_LEGACY, struct.pack("<2i", 111, 222))
    fake_hand._queue(MsgID.PRESSURE_2_LEGACY, struct.pack("<2i", 333, 444))
    bus.poll(timeout=0.01)
    assert list(bus.state.pressures) == [111, 222, 333, 444]


def test_implausible_pressures_read_as_zero(bus, fake_hand):
    fake_hand._queue(MsgID.PRESSURE_1, struct.pack("<2i", -5, 99999))
    bus.poll(timeout=0.01)
    assert list(bus.state.pressures[:2]) == [0, 0]


def test_temperatures_decode_per_finger(bus, fake_hand):
    fake_hand.temperatures[:] = np.arange(20, 36)
    fake_hand.temperature_period_ms = 1
    bus.handshake(timeout=1.0, temperature_period_ms=1)
    drain(bus)
    assert list(bus.state.temperatures) == list(range(20, 36))


def test_imu_decodes(bus, fake_hand):
    fake_hand.imu_period_ms = 1
    bus.handshake(timeout=1.0, imu_period_ms=1)
    drain(bus)
    assert list(bus.state.imu) == [1, 2, 3]


def test_error_frames_latch_per_joint(bus, fake_hand):
    fake_hand.report_error(7, int(ErrorFlag.OVERLOAD))
    bus.poll(timeout=0.01)
    assert bus.state.errors[7] == ErrorFlag.OVERLOAD
    assert bus.state.error_count == 1
    assert [e.motor_id for e in bus.state.active_errors()] == [7]

    bus.clear_errors()
    assert not bus.state.errors.any()
    assert bus.state.active_errors() == []


def test_can_error_frames_are_counted_not_decoded(bus, fake_hand):
    import can

    fake_hand._rx.append(can.Message(is_error_frame=True, arbitration_id=0, data=b""))
    bus.poll(timeout=0.01)
    assert bus.state.can_error_frames == 1


def test_malformed_frames_are_ignored(bus, fake_hand):
    fake_hand._queue(MsgID.POSITION, b"\x01")  # too short
    bus.poll(timeout=0.01)
    assert not bus.state.positions.any()


def test_poll_stops_at_max_frames(bus, fake_hand):
    for _ in range(10):
        fake_hand._queue(MsgID.PRESSURE_1, struct.pack("<2i", 1, 2))
    assert bus.poll(timeout=0.0, max_frames=4) == 4


def test_rtr_requests_go_out_and_are_answered(bus, fake_hand):
    fake_hand.sent.clear()
    bus.request_info()
    bus.request_serial()
    bus.request_positions()
    bus.request_imu()
    bus.request_temperatures()
    assert fake_hand.sent_ids() == [
        MsgID.INFO, MsgID.SERIAL,
        *[MsgID.POSITION + f for f in range(4)],
        MsgID.IMU,
        *[MsgID.TEMPERATURE + f for f in range(4)],
    ]
    bus.poll(timeout=0.01)
    assert bus.info.serial_number == "5TBR0017"


def test_diagnose_and_repr_do_not_explode(bus):
    assert "test0" in bus.diagnose()
    assert "AllegroCANBus" in repr(bus)


def test_context_manager_opens_and_closes(fake_hand):
    with AllegroCANBus("test0", bus_factory=lambda: fake_hand) as bus:
        assert bus.is_open
    assert fake_hand.closed
