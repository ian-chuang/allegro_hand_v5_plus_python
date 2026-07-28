"""
A simulated Allegro Hand V5 that speaks the real CAN protocol.

`FakeHand` implements the small part of the python-can `Bus` interface that
`AllegroCANBus` uses — `send`, `recv`, `shutdown` — so it can be handed to the
driver through `bus_factory` and the whole stack runs offline.

Joint dynamics are a velocity source, `dq/dt = gain * current`, which is enough
to make the host PD converge and to check the sign of everything.
"""

from __future__ import annotations

import struct
import time
from collections import deque

import can
import numpy as np

from allegro_hand_v5 import protocol as proto
from allegro_hand_v5.protocol import NUM_FINGERS, NUM_JOINTS, MsgID


class FakeHand:
    """Simulated hand on the other end of a CAN bus."""

    def __init__(
        self,
        serial: str = "5TBR0017",
        hardware_version: int = 0x0500,
        firmware_version: int = 0x0101,
        answer_info: bool = True,
        report_with_servo_off: bool = False,
        dynamics_gain: float = 0.01,  # rad/s per mA
        pressures=(10, 20, 30, 40),
    ):
        self.serial = serial
        self.hardware_version = hardware_version
        self.firmware_version = firmware_version
        self.answer_info = answer_info
        self.report_with_servo_off = report_with_servo_off
        self.dynamics_gain = dynamics_gain
        self.pressures = list(pressures)

        self.positions = np.zeros(NUM_JOINTS)
        self.currents = np.zeros(NUM_JOINTS)
        self.temperatures = np.full(NUM_JOINTS, 30.0)
        self.servo_on = False
        self.closed = False

        self.position_period_ms = 0
        self.imu_period_ms = 0
        self.temperature_period_ms = 0

        self.sent = []  # every (message_id, data) the host transmitted
        self.servo_commands = []  # every True/False the host asked for
        self._rx = deque()
        self._last_report = time.monotonic()
        self._last_step = time.monotonic()

    # ==================== python-can interface ====================

    def send(self, msg: "can.Message", timeout=None) -> None:
        msg_id = proto.message_id(msg.arbitration_id)
        data = bytes(msg.data)
        self.sent.append((msg_id, data))

        if msg_id == MsgID.SERVO_ON:
            self.servo_on = True
            self.servo_commands.append(True)
        elif msg_id == MsgID.SERVO_OFF:
            self.servo_on = False
            self.servo_commands.append(False)
            self.currents[:] = 0.0
        elif MsgID.SET_TORQUE <= msg_id < MsgID.SET_TORQUE + NUM_FINGERS:
            finger = msg_id - MsgID.SET_TORQUE
            self.currents[proto.finger_slice(finger)] = struct.unpack("<4h", data[:8])
        elif msg_id == MsgID.SET_PERIOD:
            self.position_period_ms, self.imu_period_ms, self.temperature_period_ms = (
                struct.unpack("<3h", data[:6]))
        elif msg_id == MsgID.INFO and msg.is_remote_frame:
            if self.answer_info:
                self._queue(MsgID.INFO, struct.pack("<2HBBB", self.hardware_version,
                                                    self.firmware_version, 0, 0, 1))
        elif msg_id == MsgID.SERIAL and msg.is_remote_frame:
            self._queue(MsgID.SERIAL, self.serial.encode("ascii").ljust(8, b"\x00"))
        elif MsgID.POSITION <= msg_id < MsgID.POSITION + NUM_FINGERS and msg.is_remote_frame:
            self._queue_position(msg_id - MsgID.POSITION)
        elif MsgID.TEMPERATURE <= msg_id < MsgID.TEMPERATURE + NUM_FINGERS:
            self._queue_temperature(msg_id - MsgID.TEMPERATURE)
        elif msg_id == MsgID.IMU:
            self._queue(MsgID.IMU, struct.pack("<3h", 1, 2, 3))
        elif msg_id == MsgID.CALIBRATE:
            self._queue(MsgID.CALIBRATE_DONE, b"")

    def recv(self, timeout=None):
        deadline = time.monotonic() + (timeout or 0.0)
        while True:
            self._step()
            if self._rx:
                return self._rx.popleft()
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.0002)

    def shutdown(self) -> None:
        self.closed = True

    # ==================== Simulation ====================

    def _queue(self, msg_id: int, data: bytes) -> None:
        self._rx.append(can.Message(arbitration_id=proto.frame_id(msg_id), data=data,
                                    is_extended_id=False))

    def _queue_position(self, finger: int) -> None:
        counts = np.round(self.positions[proto.finger_slice(finger)] / proto.POSITION_SCALE)
        self._queue(MsgID.POSITION + finger,
                    struct.pack("<4h", *counts.clip(-32768, 32767).astype(int)))

    def _queue_temperature(self, finger: int) -> None:
        values = self.temperatures[proto.finger_slice(finger)].astype(int)
        self._queue(MsgID.TEMPERATURE + finger, struct.pack("<4B", *values))

    def _step(self) -> None:
        """Advance the joints and emit whatever reports are due."""
        now = time.monotonic()
        dt, self._last_step = now - self._last_step, now
        if self.servo_on:
            self.positions += self.dynamics_gain * self.currents * dt

        if not (self.servo_on or self.report_with_servo_off):
            return
        if not self.position_period_ms:
            return
        if now - self._last_report < self.position_period_ms / 1000.0:
            return
        self._last_report = now

        for finger in range(NUM_FINGERS):
            self._queue_position(finger)
        self._queue(MsgID.PRESSURE_1, struct.pack("<2i", *self.pressures[:2]))
        self._queue(MsgID.PRESSURE_2, struct.pack("<2i", *self.pressures[2:]))
        if self.temperature_period_ms:
            for finger in range(NUM_FINGERS):
                self._queue_temperature(finger)
        if self.imu_period_ms:
            self._queue(MsgID.IMU, struct.pack("<3h", 1, 2, 3))

    # ==================== Test helpers ====================

    def sent_ids(self):
        return [msg_id for msg_id, _ in self.sent]

    def report_error(self, motor_id: int, code: int) -> None:
        self._queue(MsgID.ERROR, bytes([motor_id, code]))
