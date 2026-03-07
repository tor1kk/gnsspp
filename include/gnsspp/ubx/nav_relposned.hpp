#pragma once

#include <cstdint>
#include <vector>

namespace gnsspp {

/// Decoded UBX-NAV-RELPOSNED message: relative NED position (RTK rover).
/// Gives the position of this receiver relative to the base station.
struct NavRelPosNed {
    uint32_t itow;                 ///< GPS time of week [ms]
    uint16_t ref_station_id;       ///< Reference station ID

    double   rel_pos_n_m;          ///< Relative position North [m]
    double   rel_pos_e_m;          ///< Relative position East  [m]
    double   rel_pos_d_m;          ///< Relative position Down  [m]
    double   rel_pos_length_m;     ///< Distance to base [m]
    double   rel_pos_heading_deg;  ///< Heading to base [deg], 0..360

    float    acc_n_m;              ///< Accuracy North [m]
    float    acc_e_m;              ///< Accuracy East  [m]
    float    acc_d_m;              ///< Accuracy Down  [m]
    float    acc_length_m;         ///< Accuracy of distance [m]
    float    acc_heading_deg;      ///< Accuracy of heading [deg]

    // Flags decoded from the X4 flags field
    bool     gnss_fix_ok;          ///< Valid GNSS fix
    bool     diff_soln;            ///< Differential corrections applied
    bool     rel_pos_valid;        ///< relPos components are valid
    uint8_t  carr_soln;            ///< Carrier phase solution: 0=no, 1=float, 2=fixed
    bool     ref_pos_miss;         ///< Base station position missing
    bool     ref_obs_miss;         ///< Base station observations missing
    bool     rel_pos_heading_valid;///< relPosHeading is valid
};

/// Decode a raw UBX-NAV-RELPOSNED payload into a NavRelPosNed struct.
/// @throws gnsspp::ParseError if payload is too short.
NavRelPosNed decode_nav_relposned(const std::vector<uint8_t>& payload);

} // namespace gnsspp
