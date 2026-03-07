#include <stdexcept>
#include <string>
#include <vector>

#include "gnsspp/error.hpp"
#include "gnsspp/rtcm3/rtcm3_parser.hpp"


namespace gnsspp {

static constexpr uint8_t RTCM3_PREAMBLE = 0xD3;


bool RTCM3Parser::matches(uint8_t b1, uint8_t b2) const
{
    // b2 is the first length byte; upper 6 bits are reserved and must be 0
    return b1 == RTCM3_PREAMBLE && (b2 & 0xFC) == 0x00;
}


uint32_t RTCM3Parser::calc_crc(const uint8_t* data, size_t len)
{
    static constexpr uint32_t POLY = 0x1864CFB;
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int j = 0; j < 8; ++j) {
            crc <<= 1;
            if (crc & 0x1000000u)
                crc ^= POLY;
        }
    }
    return crc & 0xFFFFFFu;
}


Frame RTCM3Parser::parse(Port& port, uint8_t b1, uint8_t b2)
{
    // b1 == 0xD3 (preamble), b2 == upper length byte (6 reserved + 2 MSBs)
    uint8_t len_lo = port.read_byte();
    uint16_t payload_len = static_cast<uint16_t>((b2 & 0x03u) << 8) | len_lo;

    // Payload
    std::vector<uint8_t> payload(payload_len);
    size_t received = 0;
    while (received < payload_len) {
        size_t n = port.read(payload.data() + received, payload_len - received);
        if (n == 0)
            throw IoError("RTCM3: port closed during payload read");
        received += n;
    }

    // CRC-24Q: 3 bytes, big-endian
    uint8_t crc_bytes[3];
    crc_bytes[0] = port.read_byte();
    crc_bytes[1] = port.read_byte();
    crc_bytes[2] = port.read_byte();
    uint32_t rx_crc = (static_cast<uint32_t>(crc_bytes[0]) << 16) |
                      (static_cast<uint32_t>(crc_bytes[1]) << 8)  |
                       static_cast<uint32_t>(crc_bytes[2]);

    // CRC is computed over preamble + both length bytes + payload
    std::vector<uint8_t> crc_data;
    crc_data.reserve(3 + payload_len);
    crc_data.push_back(b1);
    crc_data.push_back(b2);
    crc_data.push_back(len_lo);
    crc_data.insert(crc_data.end(), payload.begin(), payload.end());

    uint32_t calc = calc_crc(crc_data.data(), crc_data.size());
    if (calc != rx_crc)
        throw ParseError("RTCM3: CRC mismatch");

    // Extract 12-bit message type from the first 12 bits of payload
    std::string type;
    if (payload_len >= 2) {
        uint16_t msg_type = (static_cast<uint16_t>(payload[0]) << 4) |
                            (payload[1] >> 4);
        type = std::to_string(msg_type);
    }

    // Assemble raw frame: preamble + len(2) + payload + CRC(3)
    Frame frame;
    frame.raw.reserve(3 + payload_len + 3);
    frame.raw.push_back(b1);
    frame.raw.push_back(b2);
    frame.raw.push_back(len_lo);
    frame.raw.insert(frame.raw.end(), payload.begin(), payload.end());
    frame.raw.push_back(crc_bytes[0]);
    frame.raw.push_back(crc_bytes[1]);
    frame.raw.push_back(crc_bytes[2]);

    frame.protocol       = "RTCM3";
    frame.type           = type;
    frame.payload_offset = 3;                              // D3 len_hi len_lo
    frame.payload_size   = payload_len;

    return frame;
}

} // namespace gnsspp
