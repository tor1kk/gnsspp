#!/usr/bin/env python3
"""
simple_reader.py — minimal example: read UBX/NMEA/RTCM3 frames from a
serial port and decode the most common message types.

Usage:
    python3 examples/simple_reader.py /dev/ttyACM0 38400
"""
import sys
import gnsspp


def handle_ubx(frame: gnsspp.Frame) -> None:
    payload = frame.payload()

    if frame.type == "NAV-PVT":
        m = gnsspp.decode_nav_pvt(payload)
        carr = ["none", "float", "fixed"][m.carr_soln]
        print(f"[NAV-PVT]  fix={m.fix_type} lat={m.lat:.7f} lon={m.lon:.7f}"
              f" alt={m.height/1000:.3f}m numSV={m.num_sv} RTK={carr}")

    elif frame.type == "NAV-HPPOSLLH":
        m = gnsspp.decode_nav_hpposllh(payload)
        if not m.invalid_llh:
            print(f"[HPPOSLLH] lat={m.lat:.9f} lon={m.lon:.9f}"
                  f" hMSL={m.h_msl/1000:.4f}m"
                  f" hAcc={m.h_acc:.1f}mm vAcc={m.v_acc:.1f}mm")

    elif frame.type == "NAV-STATUS":
        m = gnsspp.decode_nav_status(payload)
        carr = ["none", "float", "fixed"][m.carr_soln]
        print(f"[NAV-STATUS] fix={m.gps_fix} ok={m.gps_fix_ok}"
              f" diff={m.diff_soln} RTK={carr} ttff={m.ttff}ms")

    elif frame.type == "NAV-SVIN":
        m = gnsspp.decode_nav_svin(payload)
        print(f"[NAV-SVIN] dur={m.dur}s valid={m.valid} active={m.active}"
              f" acc={m.mean_acc_m*1000:.1f}mm")

    elif frame.type == "NAV-SAT":
        m = gnsspp.decode_nav_sat(payload)
        print(f"[NAV-SAT]  {m.num_svs} satellites tracked")

    elif frame.type:
        print(f"[UBX] {frame.type} ({len(payload)} bytes)")


def handle_nmea(frame: gnsspp.Frame) -> None:
    sentence = frame.raw.decode("ascii", errors="replace")

    if frame.type == "GGA":
        m = gnsspp.decode_gga(sentence)
        print(f"[GGA] fix={m.fix_quality} lat={m.lat:.7f} lon={m.lon:.7f}"
              f" alt={m.alt_m:.3f}m numSV={m.num_sv}")

    elif frame.type == "RMC":
        m = gnsspp.decode_rmc(sentence)
        if m.active:
            print(f"[RMC] lat={m.lat:.7f} lon={m.lon:.7f}"
                  f" speed={m.speed_knots:.1f}kt")


def handle_rtcm3(frame: gnsspp.Frame) -> None:
    payload = frame.payload()

    if frame.type == "1005":
        m = gnsspp.decode_msg1005(payload)
        print(f"[RTCM 1005] station={m.station_id}"
              f" X={m.ecef_x:.3f} Y={m.ecef_y:.3f} Z={m.ecef_z:.3f} m")

    elif frame.type in ("1074", "1084", "1094", "1124"):
        m = gnsspp.decode_msm4(payload)
        print(f"[RTCM {frame.type}] station={m.station_id}"
              f" sats={len(m.satellites)} signals={len(m.signals)}")


def main() -> None:
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <device> <baudrate>", file=sys.stderr)
        print(f"  e.g. {sys.argv[0]} /dev/ttyACM0 38400", file=sys.stderr)
        sys.exit(1)

    device   = sys.argv[1]
    baudrate = int(sys.argv[2])

    port = gnsspp.SerialPort(device, baudrate)
    try:
        port.open()
    except gnsspp.IoError as e:
        print(f"Failed to open {device}: {e}", file=sys.stderr)
        sys.exit(1)

    reader = gnsspp.FrameReader(port)
    reader.add_parser(gnsspp.UBXParser())
    reader.add_parser(gnsspp.NMEAParser())
    reader.add_parser(gnsspp.RTCM3Parser())

    print(f"Reading from {device} at {baudrate} baud...")

    try:
        while True:
            try:
                frame = reader.read_frame(timeout_ms=2000)
                if frame is None:
                    continue

                if   frame.protocol == "UBX":   handle_ubx(frame)
                elif frame.protocol == "NMEA":  handle_nmea(frame)
                elif frame.protocol == "RTCM3": handle_rtcm3(frame)

            except gnsspp.ParseError as e:
                print(f"[WARN] parse error: {e}", file=sys.stderr)
            except gnsspp.IoError as e:
                print(f"[ERROR] I/O error: {e}", file=sys.stderr)
                break
    except KeyboardInterrupt:
        pass
    finally:
        port.close()


if __name__ == "__main__":
    main()
