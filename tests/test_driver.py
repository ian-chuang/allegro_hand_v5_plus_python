"""`AllegroHand` end to end, control process and all, against a simulated hand."""

import time

import numpy as np
import pytest

from allegro_hand_v5 import AllegroHand, DriverConfig, Gains, HandState
from allegro_hand_v5.driver import MODE_IDLE, MODE_POSITION, _Shared
from allegro_hand_v5.exceptions import AllegroStateError
from allegro_hand_v5.gains import DEFAULT, SAFE
from allegro_hand_v5.protocol import HandInfo, NUM_JOINTS
from fake_hand import FakeHand

FAST = DriverConfig(position_period_ms=1, stale_timeout=0.05)


def wait_until(predicate, timeout=3.0, message="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    pytest.fail(f"timed out waiting for {message}")


@pytest.fixture
def hand(request):
    """A started driver wired to a simulated hand. Marker: `driver_kwargs`."""
    kwargs = dict(getattr(request, "param", {}))
    fake = kwargs.pop("fake", None) or FakeHand()
    driver = AllegroHand("test0", config=FAST, bus_factory=lambda: fake, **kwargs)
    driver.fake = fake  # the parent's copy; the child forks its own
    driver.start(timeout=5.0)
    yield driver
    driver.close()


# ==================== Lifecycle ====================


def test_start_brings_the_loop_up_and_identifies_the_hand(hand):
    assert hand.running
    assert hand.info.serial_number == "5TBR0017"
    assert hand.info.handedness == "right"
    wait_until(lambda: hand.read().updates > 5, message="control updates")
    assert hand.read().rate > 100, "the loop should follow the 1 ms position stream"


def test_close_stops_the_process_and_is_idempotent(hand):
    hand.close()
    assert not hand.running
    hand.close()


def test_commands_before_start_are_refused():
    driver = AllegroHand("test0", bus_factory=lambda: FakeHand())
    for call in (lambda: driver.set_position(np.zeros(16)),
                 lambda: driver.set_current(np.zeros(16)),
                 driver.servo_on):
        with pytest.raises(AllegroStateError):
            call()


def test_a_hand_that_never_answers_fails_to_start():
    silent = FakeHand(serial="", answer_info=False)
    silent.report_with_servo_off = False
    silent.position_period_ms = 0
    driver = AllegroHand("test0", config=DriverConfig(position_period_ms=0),
                         bus_factory=lambda: silent)
    with pytest.raises(AllegroStateError):
        driver.start(timeout=4.0)
    assert not driver.running


def test_context_manager_starts_and_closes():
    with AllegroHand("test0", config=FAST, bus_factory=lambda: FakeHand()) as hand:
        assert hand.running
        assert hand.serial_number == "5TBR0017"
    assert not hand.running


def test_start_can_leave_the_servos_off():
    driver = AllegroHand("test0", config=FAST, bus_factory=lambda: FakeHand())
    driver.start(servo=False)
    try:
        wait_until(lambda: not driver.servo_is_on, message="servos off")
    finally:
        driver.close()


# ==================== Calibration ====================


def test_the_calibration_is_chosen_by_serial_number(hand):
    assert hand.calibration.serial == "5TBR0017"
    assert hand.calibration.path is not None


@pytest.mark.parametrize("hand", [{"calibration": None}], indirect=True)
def test_calibration_can_be_disabled(hand):
    assert hand.calibration is None
    q = hand.set_position(np.full(NUM_JOINTS, 3.0))
    assert (q == 3.0).all(), "nothing should clip a target with no calibration"


def test_targets_are_clipped_into_the_calibrated_range(hand):
    q = hand.set_position(np.full(NUM_JOINTS, 10.0))
    assert q == pytest.approx(hand.calibration.upper)
    assert hand.target == pytest.approx(hand.calibration.upper)
    assert (hand.set_position(np.full(NUM_JOINTS, -10.0)) ==
            pytest.approx(hand.calibration.lower))


def test_clipping_can_be_skipped_per_call(hand):
    q = hand.set_position(np.full(NUM_JOINTS, 10.0), clip=False)
    assert (q == 10.0).all()


def test_resolving_the_calibration_does_not_move_the_held_pose(monkeypatch):
    # start() only learns the serial number, and so the homing offset, after the
    # control loop is already holding a target. That target has to move with it.
    from allegro_hand_v5.calibration import Calibration

    offset = np.full(NUM_JOINTS, 0.3)
    driver = AllegroHand("test0", config=FAST, bus_factory=lambda: FakeHand())
    monkeypatch.setattr(Calibration, "for_serial",
                        classmethod(lambda cls, serial: Calibration(offset=offset)))
    driver.start()
    try:
        assert driver.target == pytest.approx(offset, abs=0.01)
        wait_until(lambda: np.allclose(driver.positions, driver.target, atol=0.01),
                   message="the target to line up with the offset positions")
    finally:
        driver.close()


@pytest.mark.parametrize("hand", [{"calibration": None}], indirect=True)
def test_the_homing_offset_shifts_reported_positions(hand):
    from allegro_hand_v5.calibration import Calibration

    baseline = hand.positions.copy()
    offset = np.linspace(0.1, 0.5, NUM_JOINTS)
    hand.calibration = Calibration(offset=offset)
    hand._shared.write_command(offset=offset)

    wait_until(lambda: np.allclose(hand.positions, baseline + offset, atol=1e-3),
               message="offset positions")
    assert hand.raw_positions == pytest.approx(baseline, abs=1e-3)


# ==================== Control ====================


def test_idle_commands_zero_current(hand):
    wait_until(lambda: hand.read().updates > 5, message="control updates")
    state = hand.read()
    assert state.mode == "idle"
    assert (state.currents == 0).all()


@pytest.mark.parametrize("hand", [{"calibration": None}], indirect=True)
def test_position_mode_drives_the_joints_to_the_target(hand):
    target = np.full(NUM_JOINTS, 0.2)
    hand.set_position(target)
    wait_until(lambda: hand.mode == "position", message="position mode")
    wait_until(lambda: np.allclose(hand.positions, target, atol=0.005), timeout=5.0,
               message="the hand to reach its target")


@pytest.mark.parametrize("hand", [{"calibration": None}], indirect=True)
def test_velocity_is_estimated_from_the_position_stream(hand):
    # The simulated joints are a velocity source: 0.01 rad/s per mA.
    hand.set_current(np.full(NUM_JOINTS, 20.0))
    wait_until(lambda: np.allclose(hand.read().velocities, 0.2, atol=0.05), timeout=2.0,
               message="the velocity estimate to settle")


@pytest.mark.parametrize("hand", [{"calibration": None}], indirect=True)
def test_the_pd_output_is_exactly_kp_times_error_minus_kd_times_velocity(hand):
    # Nothing between the gains and the wire: no torque model, no rescaling.
    hand.set_gains(kp=100.0, kd=0.0, max_current=240.0)
    hand.set_position(np.full(NUM_JOINTS, 1.0))
    time.sleep(0.1)
    state = hand.read()
    expected = 100.0 * (1.0 - state.positions) - 0.0 * state.velocities
    assert state.currents == pytest.approx(np.clip(expected, -240, 240), abs=1.0)


@pytest.mark.parametrize("hand", [{"calibration": None}], indirect=True)
def test_current_mode_passes_the_command_through(hand):
    hand.set_current(np.full(NUM_JOINTS, 20.0))
    wait_until(lambda: hand.mode == "current", message="current mode")
    wait_until(lambda: np.allclose(hand.read().currents, 20.0), message="the commanded current")


def test_current_is_clamped_by_the_gains(hand):
    hand.set_preset(SAFE)
    hand.set_current(np.full(NUM_JOINTS, 5000.0))
    wait_until(lambda: np.allclose(hand.read().currents, SAFE.max_current),
               message="the current clamp")


def test_relax_returns_to_zero_current(hand):
    hand.set_current(np.full(NUM_JOINTS, 20.0))
    wait_until(lambda: hand.read().currents.any(), message="a non-zero current")
    hand.relax()
    wait_until(lambda: not hand.read().currents.any(), message="zero current")
    assert hand.mode == "idle"


def test_hold_freezes_at_the_measured_pose(hand):
    target = hand.hold()
    assert target == pytest.approx(hand.calibration.clip(hand.positions), abs=0.05)
    wait_until(lambda: hand.mode == "position", message="position mode")


@pytest.mark.parametrize("call,arg", [
    ("set_position", np.zeros(4)),
    ("set_current", np.zeros(20)),
])
def test_wrong_command_lengths_are_refused(hand, call, arg):
    with pytest.raises(ValueError):
        getattr(hand, call)(arg)


# ==================== Safety ====================


def test_losing_the_position_stream_forces_zero_current(hand):
    # The simulated hand only reports with the servos on, so switching them off
    # is exactly the "feedback stopped" case the watchdog exists for. Ask for
    # current anyway: the loop must refuse to act on stale positions.
    hand.servo_off()
    hand.set_current(np.full(NUM_JOINTS, 20.0))
    wait_until(lambda: hand.read().position_age > FAST.stale_timeout,
               message="the position stream to go stale")
    state = hand.read()
    assert state.mode == "idle"
    assert (state.currents == 0).all()


def test_servo_off_then_on(hand):
    hand.servo_off()
    wait_until(lambda: not hand.servo_is_on, message="servos off")
    hand.servo_on()
    wait_until(lambda: hand.servo_is_on, message="servos on")


def test_the_child_stops_when_its_parent_disappears():
    # A getppid() watchdog is what protects a hand whose owner was SIGKILLed.
    import multiprocessing as mp
    import os

    from allegro_hand_v5.driver import _control_loop

    shared = _Shared()
    shared.running[0] = 1.0
    shared.write_command(kp=np.zeros(16), kd=np.zeros(16), i_max=np.zeros(16),
                         offset=np.zeros(16), mode=MODE_IDLE, want_servo=1.0)
    proc = mp.get_context("fork").Process(
        target=_control_loop,
        args=(shared, FAST, {"channel": "test0", "bus_factory": FakeHand}, os.getpid() + 10**6),
        daemon=True,
    )
    proc.start()
    proc.join(timeout=5.0)
    assert not proc.is_alive(), "the loop must exit when its parent is gone"
    assert shared.stopped[0] > 0.5, "and must confirm it disarmed the hand"


# ==================== Gains ====================


def test_set_gains_reaches_the_control_loop(hand):
    gains = hand.set_gains(kp=123.0, kd=4.0, max_current=56.0)
    assert (gains.kp == 123.0).all()
    assert hand._shared.kp == pytest.approx(np.full(NUM_JOINTS, 123.0))
    assert hand._shared.kd == pytest.approx(np.full(NUM_JOINTS, 4.0))
    assert hand._shared.i_max == pytest.approx(np.full(NUM_JOINTS, 56.0))


def test_set_gains_leaves_untouched_fields_alone(hand):
    hand.set_preset("default")
    hand.set_gains(kp=10.0)
    assert (hand.gains.kd == DEFAULT.kd).all()


def test_set_preset_accepts_a_name_or_an_object(hand):
    assert hand.set_preset("soft").name == "soft"
    custom = Gains(kp=1.0, kd=2.0, max_current=3.0, name="mine")
    assert hand.set_preset(custom) is custom
    assert hand._shared.kp == pytest.approx(np.ones(NUM_JOINTS))


def test_gains_can_be_given_by_name_at_construction():
    driver = AllegroHand("test0", gains="compliant", bus_factory=lambda: FakeHand())
    assert driver.gains.name == "compliant"


# ==================== State ====================


def test_read_returns_a_full_snapshot(hand):
    state = hand.read()
    assert isinstance(state, HandState)
    for field, size in (("positions", 16), ("velocities", 16), ("currents", 16),
                        ("pressures", 4), ("temperatures", 16), ("imu", 3), ("errors", 16)):
        assert getattr(state, field).shape == (size,), field
    assert state.healthy
    assert state.joint_errors == []


def test_pressures_come_through(hand):
    wait_until(lambda: hand.pressures.any(), message="fingertip pressure")
    assert list(hand.pressures) == [10, 20, 30, 40]


def test_temperature_streaming_is_off_by_default_and_can_be_turned_on(hand):
    assert not hand.temperatures.any()

    driver = AllegroHand("test0", bus_factory=lambda: FakeHand(),
                         config=DriverConfig(position_period_ms=1, temperature_period_ms=5))
    driver.start()
    try:
        wait_until(lambda: driver.temperatures.any(), message="motor temperatures")
        assert (driver.temperatures == 30).all()
    finally:
        driver.close()


def test_errors_are_reported_per_joint(hand):
    assert hand.errors == []
    assert hand.read().healthy


def test_repr_says_what_it_is(hand):
    assert "AllegroHand" in repr(hand) and "running" in repr(hand)


# ==================== Shared memory ====================


def test_shared_state_roundtrips():
    shared = _Shared()
    q = np.linspace(-1, 1, NUM_JOINTS)
    shared.publish_state(t=1.5, q=q, dq=q * 2, current=q * 3,
                         pressures=np.arange(4.0), temperatures=np.zeros(NUM_JOINTS),
                         imu=np.zeros(3), errors=np.zeros(NUM_JOINTS),
                         servo_on=1.0, active_mode=MODE_POSITION,
                         position_age=0.01, updates=7, rate=333.0)
    state = shared.read_state()
    assert state["t"] == 1.5
    assert state["positions"] == pytest.approx(q)
    assert state["velocities"] == pytest.approx(q * 2)
    assert state["servo_on"] is True
    assert state["mode"] == "position"
    assert state["updates"] == 7


def test_shared_info_roundtrips():
    shared = _Shared()
    shared.publish_info(HandInfo(serial_number="5TBR0017", hardware_version=0x500,
                                 firmware_version=0x101))
    info = shared.read_info()
    assert info.serial_number == "5TBR0017"
    assert info.hardware_version == 0x500

    shared.publish_info(HandInfo(serial_number="ABC"))
    assert shared.read_info().hardware_version is None


def test_a_reader_never_sees_a_half_written_snapshot():
    shared = _Shared()
    shared.publish_state(q=np.zeros(NUM_JOINTS))
    shared.state_seq[0] += 1  # pretend a writer is mid-update
    with pytest.raises(AllegroStateError):
        shared._read(shared.state_seq, lambda: None, attempts=3)
