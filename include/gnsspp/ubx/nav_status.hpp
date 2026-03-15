#pragma once

#include <cstdint>
#include <vector>

namespace gnsspp {

/// Decoded NAV-STATUS — receiver navigation status.
struct NavStatus {
    uint32_t itow;          ///< GPS time of week, ms
    uint8_t  gps_fix;       ///< GPS fix type: 0=no fix, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
    bool     gps_fix_ok;    ///< Fix valid within limits (gpsFixOk flag)
    bool     diff_soln;     ///< Differential corrections applied
    bool     carr_soln_valid; ///< Carrier phase ranging solution valid
    uint8_t  carr_soln;     ///< 0=none, 1=float RTK, 2=fixed RTK
    uint32_t ttff;          ///< Time to first fix, ms
    uint32_t msss;          ///< Milliseconds since startup / reset
};

/// Decode a NAV-STATUS payload (must be >= 16 bytes).
/// @throws gnsspp::ParseError if payload is too short.
NavStatus decode_nav_status(const std::vector<uint8_t>& payload);

} // namespace gnsspp
