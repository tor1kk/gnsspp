#pragma once

#include "gnsspp/iparser.hpp"


namespace gnsspp {

/// Parser for NMEA 0183 sentences.
/// Frame layout: '$' <talker(2)> <type(3)> ',' <fields> '*' <cksum(2)> '\r' '\n'
class NMEAParser : public IParser {
public:
    /// Returns true for b1 == '$' (NMEA sentence start); b2 is the first
    /// talker-ID character and is not checked (can be 'G', 'P', etc.).
    bool matches(uint8_t b1, uint8_t b2) const override;

    /// Reads the rest of the sentence from @p port until '\n', verifies the
    /// XOR checksum, then dispatches to the sentence-specific decoder.
    Frame parse(Port& port, uint8_t b1, uint8_t b2) override;
};

} // namespace gnsspp
