#pragma once

#include <cstdint>
#include <string>

namespace gnsspp {

/// Decoded NMEA VTG sentence (Course Over Ground and Ground Speed).
struct NmeaVtg {
    float course_true_deg; ///< Course over ground, true north [deg]
    float course_mag_deg;  ///< Course over ground, magnetic [deg]
    float speed_knots;     ///< Speed over ground [knots]
    float speed_kmh;       ///< Speed over ground [km/h]
};

/// Decode a raw NMEA VTG sentence string into an NmeaVtg struct.
/// @throws gnsspp::ParseError on malformed or truncated sentence.
NmeaVtg decode_vtg(const std::string& sentence);

} // namespace gnsspp
