#pragma once

#include <cstdint>
#include <vector>

namespace gnsspp {

/// Decoded NAV-PVT — human-usable types after scaling and bitmask extraction.
struct NavPvt {
    uint32_t itow;          ///< GPS time of week, ms
    uint16_t year;
    uint8_t  month, day, hour, min, second;
    uint8_t  fix_type;      ///< 0=no fix, 2=2D, 3=3D, 4=GNSS+DR
    uint8_t  num_sv;        ///< SVs used in solution
    bool     gnss_fix_ok;   ///< Valid fix within DOP and accuracy limits
    uint8_t  carr_soln;     ///< 0=none, 1=float RTK, 2=fixed RTK
    double   lon;           ///< Longitude, degrees
    double   lat;           ///< Latitude, degrees
    int32_t  height;        ///< Height above ellipsoid, mm
    int32_t  h_msl;         ///< Height above MSL, mm
    uint32_t h_acc;         ///< Horizontal accuracy, mm
    uint32_t v_acc;         ///< Vertical accuracy, mm
    int32_t  vel_n;         ///< NED north velocity, mm/s
    int32_t  vel_e;         ///< NED east velocity, mm/s
    int32_t  vel_d;         ///< NED down velocity, mm/s
    int32_t  g_speed;       ///< Ground speed (2D), mm/s
    double   head_mot;      ///< Heading of motion, degrees
    uint32_t s_acc;         ///< Speed accuracy, mm/s
    double   p_dop;         ///< Position DOP
};

/// Decode a NAV-PVT payload into a NavPvt struct.
/// @param payload  Raw payload bytes from UBX frame (must be >= 92 bytes).
/// @throws gnsspp::ParseError if payload is too short.
NavPvt decode_nav_pvt(const std::vector<uint8_t>& payload);

} // namespace gnsspp
