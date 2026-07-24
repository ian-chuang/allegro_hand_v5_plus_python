"""Tests for DriverConfig: torque→current scaling, clamping, Plus handling, presets."""

import numpy as np
import pytest

from allegro_hand_v5 import DriverConfig
from allegro_hand_v5 import constants as C


def test_defaults():
    cfg = DriverConfig()
    assert cfg.nm_to_ma == C.NM_TO_MA
    assert cfg.max_current_ma.shape == (C.DOF,)
    assert np.all(cfg.max_current_ma == C.TORQUE_LIMIT_MA)


def test_torque_to_current_scale_and_clamp():
    cfg = DriverConfig()
    tau = np.zeros(C.DOF)
    tau[0] = 0.1
    cmd = cfg.torque_to_current(tau)
    assert cmd[0] == pytest.approx(0.1 * C.NM_TO_MA)
    tau[4] = 10.0  # huge -> clamp to 240 mA
    assert cfg.torque_to_current(tau)[4] == C.TORQUE_LIMIT_MA


def test_plus_halving_only_when_plus():
    cfg = DriverConfig()
    tau = np.zeros(C.DOF)
    for j in C.PLUS_HALVED_JOINTS:
        tau[j] = 0.05
    std = cfg.torque_to_current(tau, is_plus=False)
    plus = cfg.torque_to_current(tau, is_plus=True)
    for j in C.PLUS_HALVED_JOINTS:
        assert plus[j] == pytest.approx(0.5 * std[j])


def test_set_max_torque_sets_per_joint_clamp():
    cfg = DriverConfig()
    cfg.set_max_torque(0.02)
    assert np.allclose(cfg.max_current_ma, 0.02 * C.NM_TO_MA)
    tau = np.full(C.DOF, 1.0)
    assert np.all(np.abs(cfg.torque_to_current(tau)) <= 0.02 * C.NM_TO_MA + 1e-6)


def test_per_joint_and_vector_limits():
    cfg = DriverConfig()
    cfg.set_joint_max_current(3, 50)
    assert cfg.max_current_ma[3] == 50
    cfg.set_max_current(np.arange(C.DOF) + 1)
    assert cfg.max_current_ma[0] == 1 and cfg.max_current_ma[15] == 16
    with pytest.raises(ValueError):
        cfg.set_max_current([1, 2, 3])


def test_max_torque_nm_roundtrip():
    cfg = DriverConfig().set_max_torque(0.05)
    assert np.allclose(cfg.max_torque_nm, 0.05)


def test_safe_preset_is_gentle():
    cfg = DriverConfig.safe(max_torque_nm=0.02, kp_scale=0.5)
    assert np.all(cfg.max_current_ma <= C.TORQUE_LIMIT_MA)
    assert np.allclose(cfg.max_torque_nm, 0.02)
    assert cfg.kp[1] == pytest.approx(0.9 * 0.5)


def test_clamp_current_direct():
    cfg = DriverConfig().set_max_current(100)
    clamped = cfg.clamp_current(np.full(C.DOF, 500.0))
    assert np.all(clamped == 100)
