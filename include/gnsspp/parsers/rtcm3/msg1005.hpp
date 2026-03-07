#pragma once

#include <cstdint>
#include <vector>


namespace gnsspp {

/// Decoded RTCM 1005 — Stationary RTK Reference Station ARP.
struct Msg1005 {
    uint16_t station_id;  ///< Reference station ID (0-4095)
    bool     gps;         ///< GPS indicator
    bool     glonass;     ///< GLONASS indicator
    bool     galileo;     ///< Galileo indicator
    double   ecef_x;      ///< Antenna reference point ECEF X [m]
    double   ecef_y;      ///< Antenna reference point ECEF Y [m]
    double   ecef_z;      ///< Antenna reference point ECEF Z [m]
};

/// Decode an RTCM 1005 payload.
/// @param payload  Full RTCM3 payload bytes (message number bits included).
/// @throws gnsspp::ParseError if payload is too short.
Msg1005 decode_msg1005(const std::vector<uint8_t>& payload);

} // namespace gnsspp
