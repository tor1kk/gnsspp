#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace gnsspp {

/// One satellite entry within a GSV message.
struct GsvSatellite {
    uint8_t  prn;   ///< Satellite PRN / SV ID
    int8_t   elev;  ///< Elevation angle [deg], -90..90
    uint16_t azim;  ///< Azimuth angle [deg], 0..360
    uint8_t  snr;   ///< SNR / C/No [dBHz]; 0 = not being tracked
};

/// Decoded NMEA GSV sentence (Satellites in View).
/// One full GSV sequence consists of multiple messages (total_msgs in total).
struct NmeaGsv {
    uint8_t  total_msgs; ///< Total number of GSV messages in this sequence
    uint8_t  msg_num;    ///< This message's index within the sequence (1-based)
    uint8_t  total_svs;  ///< Total satellites in view (across all messages)
    uint8_t  signal_id;  ///< Signal ID (NMEA 4.11); 0 if absent
    std::vector<GsvSatellite> satellites; ///< Up to 4 satellites in this message
};

/// Decode a raw NMEA GSV sentence string into an NmeaGsv struct.
/// @throws gnsspp::ParseError on malformed or truncated sentence.
NmeaGsv decode_gsv(const std::string& sentence);

} // namespace gnsspp
