#include <stdexcept>
#include <string>

#include "gnsspp/error.hpp"
#include "gnsspp/nmea/nmea_parser.hpp"


namespace gnsspp {

static constexpr uint8_t NMEA_START   = '$';
static constexpr size_t  NMEA_MAX_LEN = 82; // NMEA 0183 max sentence length


bool NMEAParser::matches(uint8_t b1, uint8_t b2) const
{
    (void)b2; // talker-ID first char — not checked here
    return b1 == NMEA_START;
}


/// XOR of all bytes strictly between '$' and '*'.
static uint8_t calc_checksum(const std::string& sentence)
{
    uint8_t ck = 0;
    for (size_t i = 1; i < sentence.size(); ++i) {
        if (sentence[i] == '*') break;
        ck ^= static_cast<uint8_t>(sentence[i]);
    }
    return ck;
}


Frame NMEAParser::parse(Port& port, uint8_t b1, uint8_t b2)
{
    // b1 == '$', b2 == first char of talker ID (e.g. 'G')
    std::string sentence;
    sentence.reserve(NMEA_MAX_LEN);
    sentence += static_cast<char>(b1);
    sentence += static_cast<char>(b2);

    // Read bytes until '\n' or max sentence length
    while (sentence.size() < NMEA_MAX_LEN + 2) {
        uint8_t c = port.read_byte();
        sentence += static_cast<char>(c);
        if (c == '\n') break;
    }

    // Locate checksum delimiter '*'
    size_t star = sentence.rfind('*');
    if (star == std::string::npos || star + 3 > sentence.size()) {
        throw ParseError("NMEA: missing or truncated checksum");
    }

    // Decode two hex digits after '*'
    auto hex = [](char c) -> uint8_t {
        if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
        if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
        if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
        throw ParseError("NMEA: invalid checksum digit");
    };
    uint8_t expected = static_cast<uint8_t>((hex(sentence[star+1]) << 4) | hex(sentence[star+2]));
    uint8_t actual   = calc_checksum(sentence);
    if (actual != expected) {
        throw ParseError("NMEA: checksum mismatch");
    }

    // Extract 3-char sentence type from positions [3..5]:
    //   $GNGGA,...  →  [0]='$' [1]='G' [2]='N' [3]='G' [4]='G' [5]='A'
    if (sentence.size() < 7) {
        throw ParseError("NMEA: sentence header too short");
    }
    std::string type = sentence.substr(3, 3);

    Frame frame;
    frame.raw.assign(sentence.begin(), sentence.end());
    frame.protocol       = "NMEA";
    frame.type           = type;
    frame.payload_offset = 0;
    frame.payload_size   = static_cast<uint16_t>(frame.raw.size());

    return frame;
}

} // namespace gnsspp
