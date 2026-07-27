"""Exception types for the Allegro Hand V5 driver."""


class AllegroError(Exception):
    """Base class for every error this package raises."""


class AllegroConnectionError(AllegroError):
    """The CAN interface could not be opened."""


class AllegroCANError(AllegroError):
    """A CAN send or receive failed, or the bus is not open."""


class AllegroTimeoutError(AllegroError):
    """The hand did not respond in time."""


class AllegroStateError(AllegroError):
    """The driver is in the wrong state for the requested operation."""
