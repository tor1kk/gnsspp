#pragma once

/// Convenience umbrella header — includes the full gnsspp public API.
/// For minimal builds, include only the specific headers you need.

// Core
#include "gnsspp/error.hpp"
#include "gnsspp/frame.hpp"
#include "gnsspp/frame_reader.hpp"

// Ports
#include "gnsspp/ports/port.hpp"
#include "gnsspp/ports/serial_port.hpp"

// Protocol parsers (frame-level)
#include "gnsspp/parsers/iparser.hpp"
#include "gnsspp/parsers/ubx_parser.hpp"
#include "gnsspp/parsers/nmea_parser.hpp"
#include "gnsspp/parsers/rtcm3_parser.hpp"

// UBX message decoders
#include "gnsspp/parsers/ubx/nav_pvt.hpp"
#include "gnsspp/parsers/ubx/nav_sat.hpp"
#include "gnsspp/parsers/ubx/nav_svin.hpp"
#include "gnsspp/parsers/ubx/nav_relposned.hpp"
#include "gnsspp/parsers/ubx/rxm_rtcm.hpp"

// NMEA sentence decoders
#include "gnsspp/parsers/nmea/gga.hpp"
#include "gnsspp/parsers/nmea/rmc.hpp"
#include "gnsspp/parsers/nmea/gsa.hpp"
#include "gnsspp/parsers/nmea/gsv.hpp"
#include "gnsspp/parsers/nmea/vtg.hpp"

// RTCM3 message decoders
#include "gnsspp/parsers/rtcm3/msg1005.hpp"
#include "gnsspp/parsers/rtcm3/msg1006.hpp"
#include "gnsspp/parsers/rtcm3/msm4.hpp"
#include "gnsspp/parsers/rtcm3/msg1230.hpp"
