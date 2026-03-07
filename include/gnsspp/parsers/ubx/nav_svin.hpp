#pragma once

#include <cstdint>
#include <vector>

namespace gnsspp {

/// Decoded UBX-NAV-SVIN message: Time-mode survey-in status.
struct NavSvIn {
    uint32_t itow;         ///< GPS time of week [ms]
    uint32_t dur;          ///< Survey-in duration elapsed [s]
    double   mean_x_m;     ///< Mean ECEF X position [m] (cm + 0.1mm HP combined)
    double   mean_y_m;     ///< Mean ECEF Y position [m]
    double   mean_z_m;     ///< Mean ECEF Z position [m]
    float    mean_acc_m;   ///< Mean position accuracy estimate [m]
    uint32_t obs;          ///< Number of position observations used
    bool     valid;        ///< Survey-in position is valid
    bool     active;       ///< Survey-in still in progress
};

/// Decode a raw UBX-NAV-SVIN payload into a NavSvIn struct.
/// @throws gnsspp::ParseError if payload is too short.
NavSvIn decode_nav_svin(const std::vector<uint8_t>& payload);

} // namespace gnsspp
