#include "gnsspp/message.hpp"
#include "gnsspp/error.hpp"

#include "gnsspp/ubx/nav_pvt.hpp"
#include "gnsspp/ubx/nav_hpposllh.hpp"
#include "gnsspp/ubx/nav_status.hpp"
#include "gnsspp/ubx/nav_sat.hpp"
#include "gnsspp/ubx/nav_svin.hpp"
#include "gnsspp/ubx/nav_relposned.hpp"
#include "gnsspp/ubx/rxm_rtcm.hpp"
#include "gnsspp/ubx/ack.hpp"

#include "gnsspp/nmea/gga.hpp"
#include "gnsspp/nmea/rmc.hpp"
#include "gnsspp/nmea/gsa.hpp"
#include "gnsspp/nmea/gsv.hpp"
#include "gnsspp/nmea/vtg.hpp"

#include "gnsspp/rtcm3/msg1005.hpp"
#include "gnsspp/rtcm3/msg1006.hpp"
#include "gnsspp/rtcm3/msm4.hpp"
#include "gnsspp/rtcm3/msg1230.hpp"

namespace gnsspp {

std::optional<Message> decode(const Frame& frame)
{
    try {
        if (frame.protocol == "UBX") {
            auto p = frame.payload();
            if (frame.type == "NAV-PVT")       return decode_nav_pvt(p);
            if (frame.type == "NAV-HPPOSLLH")  return decode_nav_hpposllh(p);
            if (frame.type == "NAV-STATUS")    return decode_nav_status(p);
            if (frame.type == "NAV-SAT")       return decode_nav_sat(p);
            if (frame.type == "NAV-SVIN")      return decode_nav_svin(p);
            if (frame.type == "NAV-RELPOSNED") return decode_nav_relposned(p);
            if (frame.type == "RXM-RTCM")      return decode_rxm_rtcm(p);
            if (frame.type == "ACK-ACK")       return decode_ack_ack(p);
            if (frame.type == "ACK-NAK")       return decode_ack_nak(p);

        } else if (frame.protocol == "NMEA") {
            std::string s(frame.raw.begin(), frame.raw.end());
            if (frame.type == "GGA") return decode_gga(s);
            if (frame.type == "RMC") return decode_rmc(s);
            if (frame.type == "GSA") return decode_gsa(s);
            if (frame.type == "GSV") return decode_gsv(s);
            if (frame.type == "VTG") return decode_vtg(s);

        } else if (frame.protocol == "RTCM3") {
            auto p = frame.payload();
            if (frame.type == "1005") return decode_msg1005(p);
            if (frame.type == "1006") return decode_msg1006(p);
            if (frame.type == "1230") return decode_msg1230(p);
            if (frame.type == "1074" || frame.type == "1084" ||
                frame.type == "1094" || frame.type == "1124") return decode_msm4(p);
        }
    } catch (const ParseError&) {}

    return std::nullopt;
}

} // namespace gnsspp
