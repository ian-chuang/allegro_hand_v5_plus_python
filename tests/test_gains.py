"""Gain presets and the `Gains` container."""

import numpy as np
import pytest

from allegro_hand_v5 import gains as g
from allegro_hand_v5.gains import Gains
from allegro_hand_v5.protocol import NUM_JOINTS, PLUS_GEARED_JOINTS


def test_scalars_broadcast_to_every_joint():
    gains = Gains(kp=100.0, kd=5.0, max_current=200.0)
    assert gains.kp.shape == (NUM_JOINTS,)
    assert (gains.kp == 100.0).all()
    assert (gains.kd == 5.0).all()
    assert (gains.max_current == 200.0).all()


def test_vectors_are_kept_per_joint():
    kp = np.arange(NUM_JOINTS, dtype=float)
    assert (Gains(kp=kp, kd=0.0, max_current=1.0).kp == kp).all()


@pytest.mark.parametrize("bad", [np.zeros(4), np.zeros((16, 2))])
def test_wrong_shapes_are_rejected(bad):
    with pytest.raises(ValueError):
        Gains(kp=bad, kd=0.0, max_current=1.0)


@pytest.mark.parametrize("field", ["kp", "kd", "max_current"])
def test_negative_gains_are_rejected(field):
    with pytest.raises(ValueError):
        Gains(**{"kp": 1.0, "kd": 1.0, "max_current": 1.0, field: -1.0})


def test_presets_are_registered_under_their_own_name():
    for name, gains in g.PRESETS.items():
        assert gains.name == name
        assert g.preset(name) is gains
    assert set(g.PRESETS) == {"default", "compliant", "soft", "safe", "zero"}


def test_unknown_preset_lists_the_valid_names():
    with pytest.raises(KeyError, match="compliant"):
        g.preset("nope")


def test_geared_mcp2_joints_carry_half_the_gain():
    # A "Plus" hand gears joints 1/5/9 2:1, so half the current is the same
    # joint torque. The presets bake that in rather than rescaling silently.
    for gains in (g.DEFAULT, g.COMPLIANT, g.SOFT):
        for i in PLUS_GEARED_JOINTS:
            assert gains.kp[i] == pytest.approx(gains.kp[i + 1] / 2, rel=0.02)
            assert gains.max_current[i] == pytest.approx(gains.max_current[i + 1] / 2)


def test_presets_get_softer_in_order():
    assert (g.SOFT.kp < g.COMPLIANT.kp).all()
    assert (g.COMPLIANT.kp < g.DEFAULT.kp).all()
    assert (g.SAFE.max_current < g.DEFAULT.max_current).all()
    assert (g.ZERO.kp == 0).all() and (g.ZERO.kd == 0).all()


def test_presets_stay_within_the_hardware_current_limit():
    from allegro_hand_v5.protocol import MAX_CURRENT_MA

    for gains in g.PRESETS.values():
        assert (gains.max_current <= MAX_CURRENT_MA).all()


def test_default_matches_the_vendor_stiffness_after_unit_conversion():
    # libBHand runs kp = 1.0 Nm/rad on the fingers and 0.8 on the thumb, at
    # 1.43 A/Nm. See docs/bhand_gains.md.
    assert g.DEFAULT.kp[0] == pytest.approx(1.0 * 1430, rel=0.03)
    assert g.DEFAULT.kp[12] == pytest.approx(0.8 * 1430, rel=0.03)


def test_replace_overrides_only_what_it_is_given():
    tuned = g.DEFAULT.replace(kp=50.0, name="tuned")
    assert (tuned.kp == 50.0).all()
    assert (tuned.kd == g.DEFAULT.kd).all()
    assert (tuned.max_current == g.DEFAULT.max_current).all()
    assert tuned.name == "tuned"
    assert (g.DEFAULT.kp != 50.0).all(), "presets must be immutable"


def test_str_shows_every_finger():
    text = str(g.DEFAULT)
    for finger in ("index", "middle", "ring", "thumb"):
        assert finger in text
