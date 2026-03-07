#pragma once

#include <cstdint>
#include <string>

namespace gnsspp {

/// Decoded NMEA GSA sentence (DOP and Active Satellites).
struct NmeaGsa {
    bool    auto_mode;      ///< True = auto 2D/3D selection; false = manual
    uint8_t fix_type;       ///< 1 = no fix, 2 = 2D, 3 = 3D
    uint8_t sv_prns[12];    ///< PRNs of satellites used (0 = unused slot)
    uint8_t sv_count;       ///< Number of non-zero entries in sv_prns
    float   pdop;           ///< Position DOP
    float   hdop;           ///< Horizontal DOP
    float   vdop;           ///< Vertical DOP
    uint8_t system_id;      ///< GNSS system ID (NMEA 4.11; 0 if not present)
};

/// Decode a raw NMEA GSA sentence string into an NmeaGsa struct.
/// @throws gnsspp::ParseError on malformed or truncated sentence.
NmeaGsa decode_gsa(const std::string& sentence);

} // namespace gnsspp
