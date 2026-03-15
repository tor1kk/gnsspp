#pragma once

#include <optional>
#include <variant>

#include "gnsspp/frame.hpp"

// UBX
#include "gnsspp/ubx/nav_pvt.hpp"
#include "gnsspp/ubx/nav_hpposllh.hpp"
#include "gnsspp/ubx/nav_status.hpp"
#include "gnsspp/ubx/nav_sat.hpp"
#include "gnsspp/ubx/nav_svin.hpp"
#include "gnsspp/ubx/nav_relposned.hpp"
#include "gnsspp/ubx/rxm_rtcm.hpp"
#include "gnsspp/ubx/ack.hpp"

// NMEA
#include "gnsspp/nmea/gga.hpp"
#include "gnsspp/nmea/rmc.hpp"
#include "gnsspp/nmea/gsa.hpp"
#include "gnsspp/nmea/gsv.hpp"
#include "gnsspp/nmea/vtg.hpp"

// RTCM3
#include "gnsspp/rtcm3/msg1005.hpp"
#include "gnsspp/rtcm3/msg1006.hpp"
#include "gnsspp/rtcm3/msm4.hpp"
#include "gnsspp/rtcm3/msg1230.hpp"


namespace gnsspp {

/// Union of all decoded message types.
using Message = std::variant<
    // UBX — navigation
    NavPvt,
    NavHpPosLlh,
    NavStatus,
    NavSat,
    NavSvIn,
    NavRelPosNed,
    // UBX — receiver
    RxmRtcm,
    AckAck,
    AckNak,
    // NMEA
    NmeaGga,
    NmeaRmc,
    NmeaGsa,
    NmeaGsv,
    NmeaVtg,
    // RTCM3
    Msg1005,
    Msg1006,
    Msm4,
    Msg1230
>;

/// Decode a frame into a typed Message.
/// Returns std::nullopt for unknown message types or on parse error.
std::optional<Message> decode(const Frame& frame);

} // namespace gnsspp
