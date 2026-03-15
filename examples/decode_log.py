#!/usr/bin/env python3
"""
decode_log.py — decode a binary UBX log file.

Useful for post-processing recordings captured with e.g.:
    cat /dev/ttyACM0 > capture.bin

Usage:
    python3 examples/decode_log.py capture.bin
"""
import sys
import gnsspp


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <logfile>", file=sys.stderr)
        sys.exit(1)

    with open(sys.argv[1], "rb") as f:
        data = f.read()

    print(f"Decoding {len(data):,} bytes from {sys.argv[1]}\n")

    framer = gnsspp.UbxFramer()
    framer.feed(data)

    counts: dict[str, int] = {}

    while True:
        frame = framer.next_frame()
        if frame is None:
            break

        cls_id, msg_id, payload = frame
        key = f"0x{cls_id:02X}/0x{msg_id:02X}"
        counts[key] = counts.get(key, 0) + 1

        # NAV-PVT (0x01 / 0x07)
        if cls_id == 0x01 and msg_id == 0x07:
            try:
                m = gnsspp.decode_nav_pvt(payload)
                carr = ["none", "float", "fixed"][m.carr_soln]
                print(f"  NAV-PVT  lat={m.lat:.7f} lon={m.lon:.7f}"
                      f" fix={m.fix_type} RTK={carr} numSV={m.num_sv}")
            except gnsspp.ParseError as e:
                print(f"  [WARN] NAV-PVT: {e}", file=sys.stderr)

        # NAV-HPPOSLLH (0x01 / 0x14)
        elif cls_id == 0x01 and msg_id == 0x14:
            try:
                m = gnsspp.decode_nav_hpposllh(payload)
                if not m.invalid_llh:
                    print(f"  HPPOSLLH lat={m.lat:.9f} lon={m.lon:.9f}"
                          f" hAcc={m.h_acc:.1f}mm vAcc={m.v_acc:.1f}mm")
            except gnsspp.ParseError as e:
                print(f"  [WARN] HPPOSLLH: {e}", file=sys.stderr)

        # NAV-SVIN (0x01 / 0x3B)
        elif cls_id == 0x01 and msg_id == 0x3B:
            try:
                m = gnsspp.decode_nav_svin(payload)
                print(f"  NAV-SVIN dur={m.dur}s valid={m.valid}"
                      f" acc={m.mean_acc_m*1000:.1f}mm")
            except gnsspp.ParseError as e:
                print(f"  [WARN] NAV-SVIN: {e}", file=sys.stderr)

    print("\n--- UBX frame counts ---")
    for key, n in sorted(counts.items()):
        print(f"  {key}   {n:>6}")


if __name__ == "__main__":
    main()
