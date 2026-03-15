from ._gnsspp import *
from ._gnsspp import (
    Frame, SerialPort, FrameReader,
    UBXParser, NMEAParser, RTCM3Parser,
    ParseError, IoError,
)

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
