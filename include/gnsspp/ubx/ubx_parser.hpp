#pragma once

#include "gnsspp/iparser.hpp"


namespace gnsspp {

/// Parser for the u-blox UBX binary protocol.
/// Frame layout: 0xB5 0x62 | class | id | len(2 LE) | payload(len) | ck_a ck_b
class UBXParser : public IParser {
public:
    /// Returns true for b1 == 0xB5, b2 == 0x62 (UBX sync bytes).
    bool matches(uint8_t b1, uint8_t b2) const override;

    /// Expects b1 == 0xB5, b2 == 0x62; reads the rest of the frame from @p port.
    Frame parse(Port& port, uint8_t b1, uint8_t b2) override;

private:
    /// Fletcher-8 checksum over bytes [class, id, len_lo, len_hi, payload...].
    static void calc_checksum(const uint8_t* data, size_t len,
                              uint8_t& ck_a, uint8_t& ck_b);
};

} // namespace gnsspp
