#pragma once

#include <cstdint>
#include <vector>

namespace gnsspp {

/// Decoded NAV-HPPOSLLH — high-precision geodetic position (RTK output).
/// Combines the standard 1e-7 deg fields with the high-precision 1e-9 deg
/// components to give sub-millimetre lat/lon resolution.
struct NavHpPosLlh {
    uint32_t itow;          ///< GPS time of week, ms
    bool     invalid_llh;   ///< true = position invalid (do not use)
    double   lon;           ///< Longitude, degrees (1e-9 deg resolution)
    double   lat;           ///< Latitude,  degrees (1e-9 deg resolution)
    double   height;        ///< Height above ellipsoid, mm (0.1 mm resolution)
    double   h_msl;         ///< Height above MSL, mm (0.1 mm resolution)
    double   h_acc;         ///< Horizontal accuracy estimate, mm (0.1 mm resolution)
    double   v_acc;         ///< Vertical accuracy estimate,   mm (0.1 mm resolution)
};

/// Decode a NAV-HPPOSLLH payload (must be >= 36 bytes).
/// @throws gnsspp::ParseError if payload is too short.
NavHpPosLlh decode_nav_hpposllh(const std::vector<uint8_t>& payload);

} // namespace gnsspp
