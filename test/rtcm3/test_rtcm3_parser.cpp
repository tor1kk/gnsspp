#include <gtest/gtest.h>
#include <vector>
#include <stdexcept>

#include "gnsspp/parsers/rtcm3_parser.hpp"
#include "mock_port.hpp"


// CRC-24Q — same algorithm as RTCM3Parser::calc_crc
static uint32_t crc24q(const uint8_t* data, size_t len)
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

// Build a complete valid RTCM3 frame from a payload.
static std::vector<uint8_t> make_rtcm3(const std::vector<uint8_t>& payload)
{
    uint16_t len = static_cast<uint16_t>(payload.size());
    uint8_t b2   = (len >> 8) & 0x03u;  // 2 MSBs of length (reserved bits = 0)
    uint8_t b3   = len & 0xFFu;         // 8 LSBs of length

    std::vector<uint8_t> crc_data;
    crc_data.push_back(0xD3);
    crc_data.push_back(b2);
    crc_data.push_back(b3);
    crc_data.insert(crc_data.end(), payload.begin(), payload.end());

    uint32_t crc = crc24q(crc_data.data(), crc_data.size());

    std::vector<uint8_t> frame = crc_data;
    frame.push_back((crc >> 16) & 0xFFu);
    frame.push_back((crc >> 8)  & 0xFFu);
    frame.push_back( crc        & 0xFFu);
    return frame;
}

// Encode msg_type (12-bit) into the first two bytes of an RTCM3 payload.
// Remaining bytes are zeroed.
static std::vector<uint8_t> payload_with_type(uint16_t msg_type, size_t total_len = 2)
{
    std::vector<uint8_t> p(total_len, 0x00);
    p[0] = (msg_type >> 4) & 0xFF;
    p[1] = static_cast<uint8_t>((msg_type & 0x0F) << 4);
    return p;
}


// ---- Tests ----------------------------------------------------------------

// Happy path: valid frame with message type 1005 (Stationary RTK Reference)
TEST(RTCM3ParserTest, ParsesValidFrame)
{
    auto raw = make_rtcm3(payload_with_type(1005));

    // FrameReader consumes b1 and b2 before calling parse(); strip them here.
    uint8_t b1 = raw[0];
    uint8_t b2 = raw[1];
    MockPort port({raw.begin() + 2, raw.end()});

    gnsspp::RTCM3Parser parser;
    ASSERT_TRUE(parser.matches(b1, b2));

    auto frame = parser.parse(port, b1, b2);
    EXPECT_EQ(frame.protocol, "RTCM3");
    EXPECT_EQ(frame.type,     "1005");
    EXPECT_EQ(frame.raw,      raw);
}

// Happy path: message type 1074 (GPS MSM4)
TEST(RTCM3ParserTest, ParsesType1074)
{
    auto raw = make_rtcm3(payload_with_type(1074, 64));  // realistic payload size

    uint8_t b1 = raw[0];
    uint8_t b2 = raw[1];
    MockPort port({raw.begin() + 2, raw.end()});

    gnsspp::RTCM3Parser parser;
    auto frame = parser.parse(port, b1, b2);
    EXPECT_EQ(frame.protocol, "RTCM3");
    EXPECT_EQ(frame.type,     "1074");
}

// Payload shorter than 2 bytes: type field should be empty string
TEST(RTCM3ParserTest, EmptyPayloadNoType)
{
    auto raw = make_rtcm3({});  // 0-byte payload

    uint8_t b1 = raw[0];
    uint8_t b2 = raw[1];
    MockPort port({raw.begin() + 2, raw.end()});

    gnsspp::RTCM3Parser parser;
    auto frame = parser.parse(port, b1, b2);
    EXPECT_EQ(frame.protocol, "RTCM3");
    EXPECT_TRUE(frame.type.empty());
}

// CRC mismatch: last byte of CRC corrupted
TEST(RTCM3ParserTest, ThrowsOnCrcMismatch)
{
    auto raw = make_rtcm3(payload_with_type(1005));
    raw.back() ^= 0xFF;  // corrupt last CRC byte

    uint8_t b1 = raw[0];
    uint8_t b2 = raw[1];
    MockPort port({raw.begin() + 2, raw.end()});

    gnsspp::RTCM3Parser parser;
    EXPECT_THROW(parser.parse(port, b1, b2), std::runtime_error);
}

// Port closes before payload is fully delivered
TEST(RTCM3ParserTest, ThrowsOnTruncatedPort)
{
    // Build a frame that claims 10-byte payload but provide only 3 bytes + no CRC
    std::vector<uint8_t> truncated = {
        0x00, 0x0A,          // b2=0, len_lo=10 => payload_len=10
        0x3E, 0xD0, 0x00     // only 3 payload bytes
        // no CRC, no remaining payload => port will run dry
    };

    MockPort port(truncated);
    gnsspp::RTCM3Parser parser;
    EXPECT_THROW(parser.parse(port, 0xD3, 0x00), std::runtime_error);
}

// matches() checks
TEST(RTCM3ParserTest, MatchesOnlyValidPreamble)
{
    gnsspp::RTCM3Parser parser;

    EXPECT_TRUE (parser.matches(0xD3, 0x00));  // valid
    EXPECT_TRUE (parser.matches(0xD3, 0x03));  // max length MSBs, reserved=0
    EXPECT_FALSE(parser.matches(0xD3, 0x04));  // reserved bit set
    EXPECT_FALSE(parser.matches(0xB5, 0x00));  // wrong preamble (UBX)
    EXPECT_FALSE(parser.matches(0x24, 0x47));  // wrong preamble (NMEA '$')
}
