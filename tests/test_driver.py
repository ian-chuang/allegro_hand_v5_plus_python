"""Tests for the process driver's shared-memory plumbing and the PD law.

These do NOT spawn the child or touch CAN — they exercise the parent-side shared buffers
and the pure control function, both hardware-free.
"""

import numpy as np
import pytest

from allegro_hand_v5 import AllegroHand, pd_torque
from allegro_hand_v5 import constants as C
from allegro_hand_v5.driver import MODE_POSITION, MODE_TORQUE


def test_pd_torque_basic():
    kp, kd, ki = np.full(C.DOF, 1.0), np.full(C.DOF, 0.1), np.zeros(C.DOF)
    tau, err, integ = pd_torque(kp, kd, ki, np.ones(C.DOF), np.zeros(C.DOF),
                                np.zeros(C.DOF), np.zeros(C.DOF), rate_hz=333.0, i_clamp=0.05)
    assert np.allclose(err, 1.0)
    assert np.allclose(tau, 1.0)  # kp*err, zero velocity, zero ki


def test_pd_integral_clamps():
    kp = kd = np.zeros(C.DOF)
    ki = np.ones(C.DOF)
    integ = np.zeros(C.DOF)
    for _ in range(10000):
        _, _, integ = pd_torque(kp, kd, ki, np.ones(C.DOF), np.zeros(C.DOF),
                                np.zeros(C.DOF), integ, rate_hz=333.0, i_clamp=0.05)
    assert np.allclose(integ, 0.05)


def test_set_target_writes_shared_and_sets_mode():
    hand = AllegroHand(channel="can0")  # not connected; no CAN touched
    q = np.linspace(-0.1, 0.1, C.DOF)
    hand.set_target(q)
    assert np.allclose(hand.target, q)  # model None -> no clamp
    assert hand._mode.value == MODE_POSITION


def test_set_torque_and_relax_flip_mode():
    hand = AllegroHand(channel="can0")
    hand.set_torque(np.full(C.DOF, 0.01))
    assert hand._mode.value == MODE_TORQUE
    hand.set_target(np.zeros(C.DOF))
    assert hand._mode.value == MODE_POSITION
    hand.relax()
    assert hand._mode.value == MODE_TORQUE


def test_state_defaults_before_connect():
    hand = AllegroHand(channel="can0")
    assert hand.positions.shape == (C.DOF,)
    assert np.all(hand.positions == 0)
    assert hand.loop_rate_hz == 0.0
    assert hand.last_error is None
    with pytest.raises(RuntimeError):
        _ = hand.model


def test_rate_hz_from_config():
    hand = AllegroHand(channel="can0")
    assert hand.rate_hz == C.DEFAULT_CONTROL_HZ
