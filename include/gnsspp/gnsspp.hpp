#pragma once

/// Convenience umbrella header — includes the full gnsspp public API.
/// For minimal builds, include only the specific headers you need.

// Core
#include "gnsspp/error.hpp"
#include "gnsspp/frame.hpp"
#include "gnsspp/frame_reader.hpp"

// Ports
#include "gnsspp/port.hpp"
#if defined(__unix__) || defined(__APPLE__)
#  include "gnsspp/posix_serial_port.hpp"
#endif

// Protocol parsers (frame-level)
#include "gnsspp/parser.hpp"
#include "gnsspp/ubx/ubx_parser.hpp"
#include "gnsspp/nmea/nmea_parser.hpp"
#include "gnsspp/rtcm3/rtcm3_parser.hpp"

// UBX messages
#include "gnsspp/ubx/nav_pvt.hpp"
#include "gnsspp/ubx/nav_hpposllh.hpp"
#include "gnsspp/ubx/nav_status.hpp"
#include "gnsspp/ubx/nav_sat.hpp"
#include "gnsspp/ubx/nav_svin.hpp"
#include "gnsspp/ubx/nav_relposned.hpp"
#include "gnsspp/ubx/rxm_rtcm.hpp"
#include "gnsspp/ubx/cfg_valset.hpp"
#include "gnsspp/ubx/cfg_rst.hpp"
#include "gnsspp/ubx/ack.hpp"

// NMEA messages
#include "gnsspp/nmea/gga.hpp"
#include "gnsspp/nmea/rmc.hpp"
#include "gnsspp/nmea/gsa.hpp"
#include "gnsspp/nmea/gsv.hpp"
#include "gnsspp/nmea/vtg.hpp"

// RTCM3 messages
#include "gnsspp/rtcm3/msg1005.hpp"
#include "gnsspp/rtcm3/msg1006.hpp"
#include "gnsspp/rtcm3/msm4.hpp"
#include "gnsspp/rtcm3/msg1230.hpp"
