#pragma once

#include <cstdint>
#include <string>

namespace gnsspp {

/// Decoded NMEA RMC sentence (Recommended Minimum Course).
/// Complements GGA: provides speed, course over ground, and date.
struct NmeaRmc {
    std::string utc;          ///< UTC time string (HHMMSS.ss)
    bool        active;       ///< True = data valid (status 'A'); false = warning ('V')
    double      lat;          ///< Latitude  [decimal degrees], positive = North
    double      lon;          ///< Longitude [decimal degrees], positive = East
    float       speed_knots;  ///< Speed over ground [knots]
    float       course_deg;   ///< Course over ground, true [deg], 0..360
    std::string date;         ///< UTC date string (DDMMYY)
};

/// Decode a raw NMEA RMC sentence string into an NmeaRmc struct.
/// @throws gnsspp::ParseError on malformed or truncated sentence.
NmeaRmc decode_rmc(const std::string& sentence);

} // namespace gnsspp
