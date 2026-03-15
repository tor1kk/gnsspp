from ._gnsspp import *
from ._gnsspp import (
    Frame, FrameReader,
    UBXParser, NMEAParser, RTCM3Parser,
    ParseError, IoError,
)

import select as _select
import serial as _serial


class SerialPort(Port):
    """Cross-platform gnsspp.Port backed by a pyserial Serial instance.

    Can be constructed from a device path + baud rate, or from an
    already-opened serial.Serial object.

    Example::

        port = gnsspp.SerialPort("/dev/ttyACM0", 38400)
        for msg in gnsspp.frames(port):
            ...
    """

    def __init__(self, path_or_serial, baudrate: int = 0):
        super().__init__()
        if isinstance(path_or_serial, _serial.Serial):
            self._s = path_or_serial
        else:
            self._s = _serial.Serial(path_or_serial, baudrate, timeout=1)
        self._buf: bytes = b''
        self._buf_pos: int = 0

    def open(self):
        if not self._s.is_open:
            self._s.open()

    def close(self):
        self._s.close()

    def is_open(self) -> bool:
        return self._s.is_open

    def wait_readable(self, timeout_ms: int) -> bool:
        r, _, _ = _select.select([self._s.fileno()], [], [], timeout_ms / 1000.0)
        return bool(r)

    def read_byte(self) -> int:
        # Drain in_waiting bytes in one read() call to avoid one Python call per byte.
        if self._buf_pos >= len(self._buf):
            n = max(1, self._s.in_waiting)
            self._buf = self._s.read(n)
            self._buf_pos = 0
            if not self._buf:
                raise IoError("Serial read timeout")
        b = self._buf[self._buf_pos]
        self._buf_pos += 1
        return b

    def write(self, data: bytes) -> int:
        return self._s.write(data)


# ── dispatch tables ───────────────────────────────────────────────────────────

_UBX_DECODERS = {
    "NAV-PVT":       decode_nav_pvt,
    "NAV-HPPOSLLH":  decode_nav_hpposllh,
    "NAV-STATUS":    decode_nav_status,
    "NAV-SAT":       decode_nav_sat,
    "NAV-SVIN":      decode_nav_svin,
    "NAV-RELPOSNED": decode_nav_relposned,
    "RXM-RTCM":      decode_rxm_rtcm,
    "ACK-ACK":       decode_ack_ack,
    "ACK-NAK":       decode_ack_nak,
}

_NMEA_DECODERS = {
    "GGA": decode_gga,
    "RMC": decode_rmc,
    "GSA": decode_gsa,
    "GSV": decode_gsv,
    "VTG": decode_vtg,
}

_RTCM3_DECODERS = {
    "1005": decode_msg1005,
    "1006": decode_msg1006,
    "1230": decode_msg1230,
    "1074": decode_msm4,
    "1084": decode_msm4,
    "1094": decode_msm4,
    "1124": decode_msm4,
}


def decode(frame: Frame):
    """Decode a Frame into a typed message object, or None if unknown.

    Example::

        frame = reader.read_frame()
        if frame and (msg := gnsspp.decode(frame)):
            if isinstance(msg, gnsspp.NavPvt):
                print(msg.lat, msg.lon)
    """
    try:
        if frame.protocol == "UBX":
            fn = _UBX_DECODERS.get(frame.type)
            return fn(frame.payload()) if fn else None
        if frame.protocol == "NMEA":
            fn = _NMEA_DECODERS.get(frame.type)
            return fn(frame.raw.decode("ascii", errors="replace")) if fn else None
        if frame.protocol == "RTCM3":
            fn = _RTCM3_DECODERS.get(frame.type)
            return fn(frame.payload()) if fn else None
    except ParseError:
        pass
    return None


def frames(port, *, timeout_ms: int = -1):
    """Yield decoded messages from a port (UBX + NMEA + RTCM3).

    Opens the port if not already open. Skips unknown and unparseable frames.

    Example::

        port = gnsspp.SerialPort("/dev/ttyACM0", 38400)
        for msg in gnsspp.frames(port):
            match msg:
                case gnsspp.NavPvt():
                    print(msg.lat, msg.lon)
                case gnsspp.NmeaGga():
                    print(msg.alt_m)
    """
    if not port.is_open():
        port.open()

    reader = FrameReader(port)
    reader.add_parser(UBXParser())
    reader.add_parser(NMEAParser())
    reader.add_parser(RTCM3Parser())

    while True:
        try:
            frame = reader.read_frame(timeout_ms=timeout_ms)
            if frame is None:
                continue
            msg = decode(frame)
            if msg is not None:
                yield msg
        except ParseError:
            pass
