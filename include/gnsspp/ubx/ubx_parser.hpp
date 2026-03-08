#pragma once

#include "gnsspp/parser.hpp"


namespace gnsspp {

/// Parser for the u-blox UBX binary protocol.
/// Frame layout: 0xB5 0x62 | class | id | len(2 LE) | payload(len) | ck_a ck_b
class UBXParser : public Parser {
public:
    /// Returns true for b1 == 0xB5, b2 == 0x62 (UBX sync bytes).
    bool matches(uint8_t b1, uint8_t b2) const override;

    /// Expects b1 == 0xB5, b2 == 0x62; reads the rest of the frame from @p port.
    Frame parse(Port& port, uint8_t b1, uint8_t b2) override;

};

} // namespace gnsspp
