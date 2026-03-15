#!/usr/bin/env python3
"""
simple_reader.py — read UBX/NMEA/RTCM3 frames from a serial port.

Usage:
    python3 examples/simple_reader.py /dev/ttyACM0 38400
"""
import sys
import gnsspp


def main() -> None:
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <device> <baudrate>", file=sys.stderr)
        sys.exit(1)

    port = gnsspp.SerialPort(sys.argv[1], int(sys.argv[2]))
    print(f"Reading from {sys.argv[1]} at {sys.argv[2]} baud...")

    try:
        for msg in gnsspp.frames(port):
            match msg:
                case gnsspp.NavPvt():
                    carr = ["none", "float", "fixed"][msg.carr_soln]
                    print(f"[NAV-PVT]  fix={msg.fix_type}"
                          f" lat={msg.lat:.7f} lon={msg.lon:.7f}"
                          f" alt={msg.height/1000:.3f}m"
                          f" numSV={msg.num_sv} RTK={carr}")

                case gnsspp.NavHpPosLlh() if not msg.invalid_llh:
                    print(f"[HPPOSLLH] lat={msg.lat:.9f} lon={msg.lon:.9f}"
                          f" hAcc={msg.h_acc:.1f}mm vAcc={msg.v_acc:.1f}mm")

                case gnsspp.NavSvIn():
                    print(f"[NAV-SVIN] dur={msg.dur}s valid={msg.valid}"
                          f" acc={msg.mean_acc_m*1000:.1f}mm")

                case gnsspp.NavSat():
                    print(f"[NAV-SAT]  {msg.num_svs} satellites")

                case gnsspp.NmeaGga():
                    print(f"[GGA] fix={msg.fix_quality}"
                          f" lat={msg.lat:.7f} lon={msg.lon:.7f}"
                          f" alt={msg.alt_m:.3f}m numSV={msg.num_sv}")

                case gnsspp.Msg1005():
                    print(f"[RTCM 1005] station={msg.station_id}"
                          f" X={msg.ecef_x:.3f} Y={msg.ecef_y:.3f}"
                          f" Z={msg.ecef_z:.3f} m")

                case gnsspp.Msm4():
                    print(f"[RTCM {msg.msg_type}] station={msg.station_id}"
                          f" sats={len(msg.satellites)}"
                          f" signals={len(msg.signals)}")

    except gnsspp.IoError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        port.close()


if __name__ == "__main__":
    main()
