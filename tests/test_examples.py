"""
The example scripts, run end to end against a simulated hand.

`can.Bus` is replaced before the driver forks its control process, so the child
inherits the fake and the whole script runs exactly as it would on hardware.
"""

import re
import runpy
import sys
from pathlib import Path

import numpy as np
import pytest

from fake_hand import FakeHand

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"

HEALTHY_LINK = {
    "channel": "can0", "up": True, "state": "ERROR-ACTIVE", "bitrate": 1_000_000,
    "restart_ms": 100, "tx_errors": 0, "rx_errors": 0, "rx_packets": 1234,
}


@pytest.fixture
def simulated_can(monkeypatch):
    """Every `can.Bus(...)` anywhere in the stack becomes a `FakeHand`."""
    monkeypatch.setattr("can.Bus", lambda *a, **k: FakeHand())
    monkeypatch.setattr("allegro_hand_v5.link_status", lambda channel="can0": dict(HEALTHY_LINK))
    monkeypatch.setattr("allegro_hand_v5.describe_link", lambda channel="can0": "can0: up")


def run_example(name, *argv):
    sys.argv = [str(EXAMPLES / name), *argv]
    runpy.run_path(str(EXAMPLES / name), run_name="__main__")


def test_every_example_is_importable():
    scripts = sorted(p.name for p in EXAMPLES.glob("*.py"))
    assert scripts == ["calibrate.py", "current_control.py", "diagnose.py",
                       "position_control.py", "read_state.py"]
    for name in scripts:
        compile((EXAMPLES / name).read_text(), name, "exec")


def test_read_state_runs(simulated_can, capsys):
    run_example("read_state.py", "--duration", "0.3", "--rate", "20", "--temperature")
    out = capsys.readouterr().out
    assert "5TBR0017" in out
    assert "index_mcp1" in out and "thumb_ip" in out
    assert "fingertip [Pa]" in out
    assert "[degC]" in out
    assert "\033[" not in out, "no escape codes when the output is not a terminal"


def test_position_control_runs(simulated_can, capsys):
    run_example("position_control.py", "--cycles", "1", "--period", "0.3",
                "--hold", "0.05", "--gains", "compliant")
    out = capsys.readouterr().out
    assert "cycle 1/1" in out
    assert "Worst-tracking joints" in out


def test_current_control_runs(simulated_can, capsys):
    run_example("current_control.py", "--joint", "2", "--current", "40",
                "--duration", "0.3", "--rate", "50")
    out = capsys.readouterr().out
    assert "index_pip" in out
    assert "Servos off." in out


def test_current_control_pd_mode_runs(simulated_can, capsys):
    run_example("current_control.py", "--joint", "2", "--pd", "20", "--duration", "0.3")
    assert "target" in capsys.readouterr().out


def test_diagnose_passes_on_a_healthy_hand(simulated_can, capsys):
    with pytest.raises(SystemExit) as exit_info:
        run_example("diagnose.py", "--timeout", "0.5")
    assert exit_info.value.code == 0
    out = capsys.readouterr().out
    assert "All four fingers reported" in out
    assert "healthy and talking" in out


def test_diagnose_reports_a_dead_link(monkeypatch, capsys):
    monkeypatch.setattr("allegro_hand_v5.link_status", lambda channel="can0": {})
    with pytest.raises(SystemExit) as exit_info:
        run_example("diagnose.py")
    assert exit_info.value.code == 1
    assert "Does it exist?" in capsys.readouterr().out


# ==================== the live tables, rendered directly ====================


def make_state(**overrides):
    from allegro_hand_v5 import HandState

    fields = dict(
        t=1.0, positions=np.zeros(16), velocities=np.zeros(16), currents=np.zeros(16),
        pressures=np.arange(4.0), temperatures=np.full(16, 31.0), imu=np.zeros(3),
        errors=np.zeros(16), servo_on=True, mode="idle", position_age=0.001,
        updates=100, rate=333.0,
    )
    return HandState(**{**fields, **overrides})


def test_read_state_renders_a_table():
    from allegro_hand_v5 import AllegroHand, ErrorFlag
    from allegro_hand_v5.calibration import Calibration

    read_state = runpy.run_path(str(EXAMPLES / "read_state.py"))
    hand = AllegroHand("can0", bus_factory=lambda: FakeHand())
    hand.calibration = Calibration(min=np.full(16, -1.0), max=np.full(16, 1.0))
    hand._info.serial_number = "5TBR0017"

    errors = np.zeros(16)
    errors[7] = int(ErrorFlag.OVERLOAD)
    state = make_state(
        positions=np.linspace(-1.0, 1.0, 16),
        currents=hand.gains.max_current.copy(),  # every joint saturated
        errors=errors,
    )
    style = read_state["Style"](enabled=True)
    screen = read_state["render"](hand, state, style, temperature=True)

    assert "5TBR0017" in screen and "right hand, type B" in screen
    for name in ("index_mcp1", "thumb_ip"):
        assert name in screen
    assert "overload" in screen
    assert "[degC]" in screen and "  31" in screen
    assert "\033[" in screen, "colour is on"

    # One travel bar per joint, with the marker where the joint actually is:
    # these positions run from the lower limit to the upper one.
    bars = re.findall(r"\[[.|]{13}\]", re.sub(r"\033\[[0-9;]*m", "", screen))
    assert len(bars) == 16
    assert bars[0] == "[|............]"
    assert bars[-1] == "[............|]"


def test_read_state_renders_without_colour():
    from allegro_hand_v5 import AllegroHand

    read_state = runpy.run_path(str(EXAMPLES / "read_state.py"))
    hand = AllegroHand("can0", bus_factory=lambda: FakeHand())
    screen = read_state["render"](hand, make_state(), read_state["Style"](False), False)
    assert "\033[" not in screen
    # The header must survive dropping the optional temperature column.
    for heading in ("#", "joint", "pos", "vel", "current", "min", "travel", "max"):
        assert heading in screen
    assert "temp" not in screen and "[degC]" not in screen


def test_travel_bar_marks_the_position():
    read_state = runpy.run_path(str(EXAMPLES / "read_state.py"))
    plain = read_state["Style"](False)
    assert read_state["travel_bar"](0.0, plain) == "[|............]"
    assert read_state["travel_bar"](1.0, plain) == "[............|]"
    assert read_state["travel_bar"](0.5, plain) == "[......|......]"
    assert read_state["travel_bar"](float("nan"), plain) == "[             ]"
    # Out of range still draws, clamped to the end of the track.
    assert read_state["travel_bar"](5.0, plain) == "[............|]"


# ==================== calibrate.py, which needs a terminal ====================


def test_calibrate_renders_a_screen():
    from allegro_hand_v5 import HandState
    from allegro_hand_v5.calibration import Calibration

    calibrate = runpy.run_path(str(EXAMPLES / "calibrate.py"))
    cal = Calibration(serial="5TBR0017")
    cal.offset[3] = 0.25
    raw = np.linspace(-0.5, 0.5, 16)
    state = HandState(
        t=1.0, positions=raw, velocities=np.zeros(16), currents=np.zeros(16),
        pressures=np.arange(4.0), temperatures=np.zeros(16), imu=np.zeros(3),
        errors=np.zeros(16), servo_on=True, mode="idle", position_age=0.001,
        updates=100, rate=333.0,
    )
    screen = calibrate["render"](cal, raw, 3, state, "hello")

    assert "5TBR0017" in screen and "hello" in screen
    assert "> thumb_dip" not in screen
    assert "> index_dip" in screen, "the selected joint is marked"
    assert "0.250" in screen, "the offset is shown"
    for name in ("index_mcp1", "thumb_ip"):
        assert name in screen
