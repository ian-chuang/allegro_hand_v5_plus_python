"""Joint limits, homing offsets, and the calibration file format."""

import json

import numpy as np
import pytest

from allegro_hand_v5.calibration import (
    CALIBRATION_DIR,
    NOMINAL_MAX,
    NOMINAL_MIN,
    Calibration,
    load_calibration,
)
from allegro_hand_v5.protocol import JOINT_NAMES, NUM_JOINTS


@pytest.fixture
def cal():
    return Calibration(
        min=np.full(NUM_JOINTS, -1.0),
        max=np.full(NUM_JOINTS, 1.0),
        serial="TEST0001",
    )


def test_defaults_to_the_nominal_range():
    cal = Calibration()
    assert (cal.min == NOMINAL_MIN).all()
    assert (cal.max == NOMINAL_MAX).all()
    assert (cal.offset == 0).all()
    assert cal.path is None


def test_wrong_length_is_rejected():
    with pytest.raises(ValueError):
        Calibration(min=np.zeros(4))


def test_limits_and_center(cal):
    assert (cal.lower == -1.0).all()
    assert (cal.upper == 1.0).all()
    assert (cal.center == 0.0).all()


def test_limits_survive_a_reversed_encoder():
    cal = Calibration(min=np.full(NUM_JOINTS, 1.0), max=np.full(NUM_JOINTS, -1.0))
    assert (cal.lower == -1.0).all()
    assert (cal.upper == 1.0).all()


def test_offset_shifts_readings_and_limits_together(cal):
    cal.offset[:] = 0.5
    assert (cal.apply(np.zeros(NUM_JOINTS)) == 0.5).all()
    assert (cal.lower == -0.5).all()
    assert (cal.upper == 1.5).all()
    # A joint sitting at its recorded maximum still reads as the upper limit.
    assert cal.apply(cal.max)[0] == pytest.approx(cal.upper[0])


def test_clip_holds_targets_inside_the_range(cal):
    clipped = cal.clip(np.full(NUM_JOINTS, 5.0))
    assert (clipped == 1.0).all()
    assert (cal.clip(np.full(NUM_JOINTS, -5.0)) == -1.0).all()
    assert (cal.clip(np.zeros(NUM_JOINTS)) == 0.0).all()


def test_normalize_and_denormalize_are_inverses(cal):
    cal.offset[:] = 0.25
    q = np.linspace(-0.9, 0.9, NUM_JOINTS) + 0.25
    assert cal.denormalize(cal.normalize(q)) == pytest.approx(q)
    assert cal.normalize(cal.lower) == pytest.approx(np.zeros(NUM_JOINTS))
    assert cal.normalize(cal.upper) == pytest.approx(np.ones(NUM_JOINTS))


def test_normalize_survives_a_zero_width_joint():
    cal = Calibration(min=np.zeros(NUM_JOINTS), max=np.zeros(NUM_JOINTS))
    assert np.isfinite(cal.normalize(np.zeros(NUM_JOINTS))).all()


def test_dict_roundtrip_keeps_every_joint(cal):
    cal.offset[3] = 0.125
    data = cal.to_dict()
    assert set(data["joints"]) == set(JOINT_NAMES)
    restored = Calibration.from_dict(data)
    assert restored.min == pytest.approx(cal.min)
    assert restored.max == pytest.approx(cal.max)
    assert restored.offset == pytest.approx(cal.offset)
    assert restored.serial == cal.serial


def test_from_dict_rejects_an_unknown_joint_name():
    with pytest.raises(ValueError, match="unknown joint"):
        Calibration.from_dict({"joints": {"pinky_dip": {"min": 0, "max": 1}}})


def test_from_dict_defaults_a_missing_offset_to_zero():
    cal = Calibration.from_dict({"joints": {"index_pip": {"min": -1, "max": 2}}})
    assert cal.offset[2] == 0.0
    assert cal.min[2] == -1 and cal.max[2] == 2


def test_save_and_load_roundtrip(tmp_path, cal):
    cal.offset[0] = -0.5
    path = cal.save(tmp_path / "TEST0001.json")
    assert path.is_file()
    assert cal.path == path

    loaded = Calibration.load(path)
    assert loaded.serial == "TEST0001"
    assert loaded.offset[0] == pytest.approx(-0.5)
    assert loaded.path == path
    # Readable and hand-editable: joint names as keys, radians as values.
    document = json.loads(path.read_text())
    assert document["units"] == "radians"
    assert document["joints"]["index_mcp1"]["min"] == pytest.approx(-1.0)


def test_save_without_a_serial_or_path_is_an_error():
    with pytest.raises(ValueError, match="serial"):
        Calibration().save()


def test_the_bundled_calibration_loads():
    cal = Calibration.for_serial("5TBR0017")
    assert cal.path == CALIBRATION_DIR / "5TBR0017.json"
    assert cal.serial == "5TBR0017"
    assert (cal.upper > cal.lower).all()
    # This hand's thumb sits well outside the nominal range; that is the point.
    assert cal.lower[13] > 1.0


def test_an_unknown_serial_falls_back_to_nominal_limits():
    cal = Calibration.for_serial("NOSUCH99")
    assert cal.path is None
    assert (cal.min == NOMINAL_MIN).all()
    assert cal.serial == "NOSUCH99"


def test_load_calibration_accepts_every_spelling(tmp_path, cal):
    path = cal.save(tmp_path / "TEST0001.json")
    assert load_calibration(None) is None
    assert load_calibration(False) is None
    assert load_calibration(cal) is cal
    assert load_calibration(str(path)).serial == "TEST0001"
    assert load_calibration(True, serial="5TBR0017").serial == "5TBR0017"


def test_summary_lists_every_joint(cal):
    text = cal.summary()
    for name in JOINT_NAMES:
        assert name in text
