#pragma once

#include <cstdint>

#include "gnsspp/parser.hpp"


namespace gnsspp {

/// Parser for the RTCM 3.x binary protocol.
/// Frame layout: 0xD3 | 0b000000xx xxxxxxxx (6 reserved + 10-bit length) |
///               payload(length) | CRC-24Q(3)
class RTCM3Parser : public Parser {
public:
    /// Returns true for b1 == 0xD3 and (b2 & 0xFC) == 0x00 (reserved bits zero).
    bool matches(uint8_t b1, uint8_t b2) const override;

    /// Reads the rest of the frame from @p port; verifies CRC-24Q.
    Frame parse(Port& port, uint8_t b1, uint8_t b2) override;

private:
    /// CRC-24Q over @p len bytes.
    static uint32_t calc_crc(const uint8_t* data, size_t len);
};

} // namespace gnsspp
