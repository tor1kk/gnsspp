#include <cstdio>
#include <stdexcept>
#include <vector>

#include "gnsspp/error.hpp"
#include "gnsspp/ubx/ubx_parser.hpp"
#include "nav_parser.hpp"
#include "rxm_parser.hpp"

namespace gnsspp {

// UBX sync chars
static constexpr uint8_t SYNC1 = 0xB5;
static constexpr uint8_t SYNC2 = 0x62;

// UBX message classes
static constexpr uint8_t CLASS_NAV = 0x01;
static constexpr uint8_t CLASS_RXM = 0x02;
static constexpr uint8_t CLASS_ACK = 0x05;


bool UBXParser::matches(uint8_t b1, uint8_t b2) const
{
    return b1 == SYNC1 && b2 == SYNC2;
}



Frame UBXParser::parse(Port& port, uint8_t b1, uint8_t b2)
{
    // b1 == SYNC1 (0xB5), b2 == SYNC2 (0x62) — already verified by matches()

    // Header: class (1) + id (1) + length (2, little-endian)
    uint8_t msg_class = port.read_byte();
    uint8_t msg_id    = port.read_byte();
    uint8_t len_lo    = port.read_byte();
    uint8_t len_hi    = port.read_byte();

    uint16_t payload_len = static_cast<uint16_t>(len_lo) |
                           (static_cast<uint16_t>(len_hi) << 8);

    // Payload
    std::vector<uint8_t> payload(payload_len);
    size_t received = 0;
    while (received < payload_len) {
        size_t n = port.read(payload.data() + received, payload_len - received);
        if (n == 0) {
            throw IoError("UBX: port closed during payload read");
        }
        received += n;
    }

    // Checksum bytes
    uint8_t rx_ck_a = port.read_byte();
    uint8_t rx_ck_b = port.read_byte();

    // Verify Fletcher-8 checksum over [class, id, len_lo, len_hi, payload...]
    uint8_t calc_a = 0, calc_b = 0;
    for (uint8_t b : {msg_class, msg_id, len_lo, len_hi}) {
        calc_a += b;
        calc_b += calc_a;
    }
    for (uint8_t b : payload) {
        calc_a += b;
        calc_b += calc_a;
    }

    if (calc_a != rx_ck_a || calc_b != rx_ck_b) {
        throw ParseError("UBX: checksum mismatch");
    }

    // Assemble raw frame
    Frame frame;
    frame.raw.reserve(6 + payload_len + 2);
    frame.raw.push_back(b1);
    frame.raw.push_back(b2);
    frame.raw.push_back(msg_class);
    frame.raw.push_back(msg_id);
    frame.raw.push_back(len_lo);
    frame.raw.push_back(len_hi);
    frame.raw.insert(frame.raw.end(), payload.begin(), payload.end());
    frame.raw.push_back(rx_ck_a);
    frame.raw.push_back(rx_ck_b);

    frame.protocol       = "UBX";
    frame.payload_offset = 6;                              // B5 62 class id len_lo len_hi
    frame.payload_size   = payload_len;

    // Dispatch to class-level parser (sets frame.type)
    switch (msg_class) {
        case CLASS_NAV: nav_parse(msg_id, frame); break;
        case CLASS_RXM: rxm_parse(msg_id, frame); break;
        case CLASS_ACK:
            if      (msg_id == 0x01) frame.type = "ACK-ACK";
            else if (msg_id == 0x00) frame.type = "ACK-NAK";
            break;
        default:
            break;  // unknown class
    }

    return frame;
}

} // namespace gnsspp
