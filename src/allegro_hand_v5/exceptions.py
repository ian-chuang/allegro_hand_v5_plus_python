"""Exception classes for the Allegro Hand V5 driver."""


class AllegroError(Exception):
    """Base exception for all Allegro Hand errors."""


class AllegroConnectionError(AllegroError):
    """Raised when connection to the hand fails."""


class AllegroCANError(AllegroError):
    """Raised when CAN communication fails."""


class AllegroBHandError(AllegroError):
    """Raised when a libBHand operation fails."""


class AllegroTimeoutError(AllegroError):
    """Raised when an operation times out."""


class AllegroStateError(AllegroError):
    """Raised when the hand is in an invalid state for the requested operation."""
