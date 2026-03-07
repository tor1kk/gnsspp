#pragma once

#include <cstdint>
#include <vector>

#include "gnsspp/parsers/rtcm3/msg1005.hpp"


namespace gnsspp {

/// Decoded RTCM 1006 — Stationary RTK Reference Station ARP with Antenna Height.
/// Superset of Msg1005.
struct Msg1006 : Msg1005 {
    double antenna_height;  ///< Antenna height above marker [m]
};

/// Decode an RTCM 1006 payload.
/// @param payload  Full RTCM3 payload bytes (message number bits included).
/// @throws gnsspp::ParseError if payload is too short.
Msg1006 decode_msg1006(const std::vector<uint8_t>& payload);

} // namespace gnsspp
