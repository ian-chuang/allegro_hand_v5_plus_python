"""Tests for per-configuration joint limits and handedness resolution."""

import numpy as np
import pytest

from allegro_hand_v5 import constants as C
from allegro_hand_v5.model import HandModel


def test_all_four_configs_present_and_shaped():
    for key in ("right_A", "right_B", "left_A", "left_B"):
        lower, upper = C.JOINT_LIMITS[key]
        assert len(lower) == C.DOF and len(upper) == C.DOF
        assert np.all(np.array(upper) > np.array(lower))


def test_default_is_right_b_plus():
    m = HandModel.default()
    assert m.config_key == "right_B"
    assert m.is_plus is True


def test_right_b_matches_measured_thumb_cmc2_offset():
    m = HandModel("5TBR0017", is_right=True, is_type_a=False)
    lo = np.degrees(m.joint_limits_lower)
    hi = np.degrees(m.joint_limits_upper)
    # thumb CMC-2 (idx 13): measured ~78.3..190.5 deg (offset but ~112 deg range)
    assert lo[13] == pytest.approx(78.320, abs=1e-2)
    assert hi[13] == pytest.approx(190.520, abs=1e-2)
    assert (hi[13] - lo[13]) == pytest.approx(112.2, abs=0.1)


def test_left_mirrors_abduction_joints_only():
    r = HandModel("R", is_right=True, is_type_a=False)
    l = HandModel("L", is_right=False, is_type_a=False)
    for j in C.ABDUCTION_JOINTS:
        # mirrored: left.lower == -right.upper, left.upper == -right.lower
        assert l.joint_limits_lower[j] == pytest.approx(-r.joint_limits_upper[j])
        assert l.joint_limits_upper[j] == pytest.approx(-r.joint_limits_lower[j])
    # a flexion joint (index MCP-2) is unchanged by the mirror
    assert l.joint_limits_lower[1] == pytest.approx(r.joint_limits_lower[1])


def test_type_a_widens_mcp2_lower():
    a = HandModel("A", is_right=True, is_type_a=True)
    b = HandModel("B", is_right=True, is_type_a=False)
    for j in C.MCP2_JOINTS:
        assert np.degrees(a.joint_limits_lower[j]) == pytest.approx(-10.0)
        assert a.joint_limits_lower[j] <= b.joint_limits_lower[j]


def test_clamp_positions_respects_limits():
    m = HandModel.default()
    clamped = m.clamp_positions(np.full(C.DOF, 10.0))  # way past every upper limit
    assert np.allclose(clamped, m.joint_limits_upper)
    clamped_lo = m.clamp_positions(np.full(C.DOF, -10.0))
    assert np.allclose(clamped_lo, m.joint_limits_lower)
