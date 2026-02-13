"""
ST3215 TTL serial servo protocol (1 Mbps).

Preset dictionary for arm poses (bumper, tenhut, lookup, lookout, reach).
Bearing-to-pan mapping for head orientation.
Auto-enters simulation mode if port cannot be opened or pyserial is missing.
"""

import threading
from typing import Any, Dict, List, Optional

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False
    serial = None  # type: ignore

# Bearing name -> pan position (0-4095, center 2048)
BEARING_PAN = {
    "forward": 2048,
    "leftish": 2560,
    "left": 3072,
    "full-left": 3584,
    "back-left": 4096 % 4096,  # 0
    "back": 2048,  # flip: same as forward, arm may flip
    "rightish": 1536,
    "right": 1024,
    "full-right": 512,
    "back-right": 0,
}

# Default presets: [pan, tilt] in 0-4095 (center 2048)
DEFAULT_PRESETS: Dict[str, List[int]] = {
    "bumper": [2048, 3072],
    "tenhut": [2048, 2048],
    "lookup": [2048, 1024],
    "lookout": [2048, 1536],
    "reach": [2048, 512],
}


class ST3215Serial:
    """
    Thread-safe ST3215 serial interface with simulation fallback.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 1000000,
        simulate: bool = False,
        presets: Optional[Dict[str, List[int]]] = None,
    ):
        self._port = port
        self._baudrate = baudrate
        self._simulate = simulate
        self._lock = threading.Lock()
        self._ser: Optional[Any] = None
        self._presets = dict(presets) if presets else dict(DEFAULT_PRESETS)
        self._positions: Dict[int, int] = {}  # servo_id -> position (0-4095)

        if not simulate and _SERIAL_AVAILABLE:
            try:
                self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.02)
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

    def set_preset(self, name: str, bearing: Optional[str] = None) -> bool:
        """
        Set servos to a named preset. If bearing is given, override pan from BEARING_PAN.
        Returns True if preset was applied.
        """
        preset = self._presets.get(name.lower())
        if not preset:
            return False
        pan, tilt = preset[0], preset[1]
        if bearing:
            pan = BEARING_PAN.get(bearing.lower(), pan)
        with self._lock:
            if self._simulate:
                self._positions[1] = pan
                self._positions[2] = tilt
                return True
            # Assume servo_ids 1=pan, 2=tilt
            self._set_position_locked(1, pan)
            self._set_position_locked(2, tilt)
        return True

    def set_position(self, servo_id: int, position: int) -> None:
        """Set one servo to position (0-4095)."""
        position = max(0, min(4095, position))
        with self._lock:
            self._positions[servo_id] = position
            if not self._simulate and self._ser and self._ser.is_open:
                self._set_position_locked(servo_id, position)

    def _set_position_locked(self, servo_id: int, position: int) -> None:
        """Must be called with _lock held. Send position to servo (simplified TTL protocol)."""
        if not self._ser or not self._ser.is_open:
            return
        # Minimal ST3215-style packet: header, id, position low, position high, etc.
        # Simplified: many ST3215 clones use a simple serial protocol; adjust to match hardware.
        position = max(0, min(4095, position))
        low = position & 0xFF
        high = (position >> 8) & 0xFF
        # Placeholder packet format - replace with actual ST3215 protocol if needed
        packet = bytes([0xFF, 0xFF, servo_id & 0xFF, 4, 0x02, low, high])
        try:
            self._ser.write(packet)
        except Exception:
            pass

    def read_feedback(self, servo_id: int) -> Optional[int]:
        """Read current position of one servo (0-4095). Returns None on error or in sim (last set)."""
        with self._lock:
            if self._simulate:
                return self._positions.get(servo_id, 2048)
            if not self._ser or not self._ser.is_open:
                return self._positions.get(servo_id)
            # Request position packet and parse response - hardware dependent
            try:
                # Placeholder: real implementation would send read command and parse reply
                return self._positions.get(servo_id, 2048)
            except Exception:
                return self._positions.get(servo_id)

    def close(self) -> None:
        """Close the serial port."""
        with self._lock:
            if self._ser and getattr(self._ser, "is_open", False):
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
