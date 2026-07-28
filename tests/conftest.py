import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))

from fake_hand import FakeHand  # noqa: E402


@pytest.fixture
def fake_hand():
    """A simulated hand streaming positions every millisecond."""
    return FakeHand()


@pytest.fixture
def bus(fake_hand):
    """An `AllegroCANBus` wired to `fake_hand`, already open."""
    from allegro_hand_v5 import AllegroCANBus

    bus = AllegroCANBus("test0", bus_factory=lambda: fake_hand)
    bus.open()
    yield bus
    bus.close()
