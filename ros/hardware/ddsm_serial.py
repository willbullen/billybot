"""
DDSM210 UART JSON protocol (115200 baud via DDSM Driver HAT).

Sends speed commands as JSON: {"T":1, "L":<left_rpm_x10>, "R":<right_rpm_x10>}
Reads encoder/current/temperature feedback from HAT.
Auto-enters simulation mode if port cannot be opened or pyserial is missing.
Thread-safe via Lock. Speed values clamped to [-2100, 2100] (0.1 rpm units).
"""

import json
import threading
import time
from typing import Any, Dict, Optional

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False
    serial = None  # type: ignore


class DDSMSerial:
    """
    Thread-safe DDSM210 serial interface with simulation fallback.
    """

    # Speed limits in 0.1 RPM units (+-210 RPM range)
    _MIN_SPEED = -2100
    _MAX_SPEED = 2100

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        simulate: bool = False,
    ):
        self._port = port
        self._baudrate = baudrate
        self._simulate = simulate
        self._lock = threading.Lock()
        self._ser: Optional[Any] = None
        self._last_left_rpm = 0.0
        self._last_right_rpm = 0.0
        # Simulated encoder counters (accumulate based on commanded speed)
        self._sim_left_encoder = 0
        self._sim_right_encoder = 0
        self._sim_last_time = time.monotonic()

        if not simulate and _SERIAL_AVAILABLE:
            try:
                self._ser = serial.Serial(
                    port=port, baudrate=baudrate, timeout=0.1
                )
            except Exception:
                self._simulate = True
                self._ser = None
        else:
            if not _SERIAL_AVAILABLE and not simulate:
                self._simulate = True
            self._ser = None

    @property
    def simulate(self) -> bool:
        return self._simulate

    def send_speed(self, left_rpm: float, right_rpm: float) -> None:
        """
        Send speed command to left and right motor pairs.

        Args:
            left_rpm: Left motor speed in RPM.
            right_rpm: Right motor speed in RPM.
        """
        # Convert RPM to 0.1 RPM units and clamp
        left_val = int(max(self._MIN_SPEED, min(self._MAX_SPEED, left_rpm * 10)))
        right_val = int(max(self._MIN_SPEED, min(self._MAX_SPEED, right_rpm * 10)))

        with self._lock:
            self._last_left_rpm = left_rpm
            self._last_right_rpm = right_rpm

            if self._simulate:
                return

            if not self._ser or not self._ser.is_open:
                return

            cmd = json.dumps({"T": 1, "L": left_val, "R": right_val})
            try:
                self._ser.write((cmd + "\n").encode("ascii"))
                self._ser.flush()
            except Exception:
                pass

    def read_feedback(self) -> Dict[str, float]:
        """
        Read motor feedback (speed, current, temperature, encoder counts).

        Returns a dict with keys: left_speed, right_speed, left_current,
        right_current, left_temp, right_temp, left_encoder, right_encoder.
        """
        with self._lock:
            if self._simulate:
                return self._sim_feedback()

            if not self._ser or not self._ser.is_open:
                return self._sim_feedback()

            try:
                # Request feedback from HAT
                self._ser.write(b'{"T":2}\n')
                self._ser.flush()
                line = self._ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    data = json.loads(line)
                    return {
                        "left_speed": data.get("LS", 0.0),
                        "right_speed": data.get("RS", 0.0),
                        "left_current": data.get("LC", 0.0),
                        "right_current": data.get("RC", 0.0),
                        "left_temp": data.get("LT", 0.0),
                        "right_temp": data.get("RT", 0.0),
                        "left_encoder": data.get("LE", 0),
                        "right_encoder": data.get("RE", 0),
                    }
            except Exception:
                pass

            return self._sim_feedback()

    def _sim_feedback(self) -> Dict[str, float]:
        """Generate simulated feedback from commanded speeds."""
        now = time.monotonic()
        dt = now - self._sim_last_time
        self._sim_last_time = now

        # Simulate encoder accumulation: rpm -> counts/sec -> counts
        # counts_per_rev = 4096, rpm / 60 = rps, rps * cpr = counts/sec
        cpr = 4096
        left_cps = (self._last_left_rpm / 60.0) * cpr
        right_cps = (self._last_right_rpm / 60.0) * cpr
        self._sim_left_encoder += int(left_cps * dt)
        self._sim_right_encoder += int(right_cps * dt)

        return {
            "left_speed": self._last_left_rpm,
            "right_speed": self._last_right_rpm,
            "left_current": 0.0,
            "right_current": 0.0,
            "left_temp": 25.0,
            "right_temp": 25.0,
            "left_encoder": self._sim_left_encoder,
            "right_encoder": self._sim_right_encoder,
        }

    def stop(self) -> None:
        """Send zero speed to both motors."""
        self.send_speed(0.0, 0.0)

    def close(self) -> None:
        """Stop motors and close the serial port."""
        self.stop()
        with self._lock:
            if self._ser and getattr(self._ser, "is_open", False):
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
