#pragma once

#include <cstdint>
#include <string>

namespace gnsspp {

/// Decoded NMEA GGA sentence (Global Positioning System Fix Data).
struct NmeaGga {
    std::string utc;          ///< UTC time string (HHMMSS.ss)
    double      lat;          ///< Latitude  [decimal degrees], positive = North
    double      lon;          ///< Longitude [decimal degrees], positive = East
    uint8_t     fix_quality;  ///< 0=invalid, 1=GPS SPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    uint8_t     num_sv;       ///< Number of satellites in use
    float       hdop;         ///< Horizontal dilution of precision
    float       alt_m;        ///< Antenna altitude above MSL [m]
    float       geoid_sep_m;  ///< Geoid separation (undulation) [m]
};

/// Decode a raw NMEA GGA sentence into an NmeaGga struct.
/// @param sentence  Complete sentence string including leading '$' and trailing '\r\n'.
/// @throws gnsspp::ParseError on malformed or truncated sentence.
NmeaGga decode_gga(const std::string& sentence);

} // namespace gnsspp
