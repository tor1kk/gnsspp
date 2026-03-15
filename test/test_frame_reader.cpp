#include <gtest/gtest.h>
#include <memory>

#include "gnsspp/error.hpp"
#include "gnsspp/frame_reader.hpp"
#include "gnsspp/ubx/ubx_parser.hpp"
#include "gnsspp/nmea/nmea_parser.hpp"
#include "gnsspp/rtcm3/rtcm3_parser.hpp"
#include "mock_port.hpp"


// ---- UBX frame builder ------------------------------------------------------

// Fletcher-8 checksum over [class, id, len_lo, len_hi, payload...]
static std::pair<uint8_t, uint8_t> ubx_checksum(
    uint8_t cls, uint8_t id,
    const std::vector<uint8_t>& payload)
{
    uint8_t a = 0, b = 0;
    for (uint8_t byte : {cls, id,
                         static_cast<uint8_t>(payload.size() & 0xFF),
                         static_cast<uint8_t>(payload.size() >> 8)}) {
        a += byte; b += a;
    }
    for (uint8_t byte : payload) { a += byte; b += a; }
    return {a, b};
}

static std::vector<uint8_t> make_ubx(uint8_t cls, uint8_t id,
                                     std::vector<uint8_t> payload = {})
{
    std::vector<uint8_t> frame = {
        0xB5, 0x62, cls, id,
        static_cast<uint8_t>(payload.size() & 0xFF),
        static_cast<uint8_t>(payload.size() >> 8),
    };
    frame.insert(frame.end(), payload.begin(), payload.end());
    auto [ck_a, ck_b] = ubx_checksum(cls, id, payload);
    frame.push_back(ck_a);
    frame.push_back(ck_b);
    return frame;
}

// ---- RTCM3 frame builder ----------------------------------------------------

static uint32_t crc24q(const uint8_t* data, size_t len)
{
    static constexpr uint32_t POLY = 0x1864CFB;
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int j = 0; j < 8; ++j) {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= POLY;
        }
    }
    return crc & 0xFFFFFFu;
}

static std::vector<uint8_t> make_rtcm3(uint16_t msg_type, size_t payload_len = 4)
{
    std::vector<uint8_t> payload(payload_len, 0);
    if (payload_len >= 2) {
        payload[0] = (msg_type >> 4) & 0xFF;
        payload[1] = static_cast<uint8_t>((msg_type & 0x0F) << 4);
    }
    std::vector<uint8_t> crc_input = {
        0xD3,
        static_cast<uint8_t>((payload_len >> 8) & 0x03),
        static_cast<uint8_t>(payload_len & 0xFF),
    };
    crc_input.insert(crc_input.end(), payload.begin(), payload.end());
    uint32_t crc = crc24q(crc_input.data(), crc_input.size());
    crc_input.push_back((crc >> 16) & 0xFF);
    crc_input.push_back((crc >> 8)  & 0xFF);
    crc_input.push_back( crc        & 0xFF);
    return crc_input;
}


// ---- Helpers ----------------------------------------------------------------

static gnsspp::FrameReader make_reader(MockPort& port)
{
    gnsspp::FrameReader reader(port);
    reader.add_parser(std::make_unique<gnsspp::UBXParser>());
    reader.add_parser(std::make_unique<gnsspp::NMEAParser>());
    reader.add_parser(std::make_unique<gnsspp::RTCM3Parser>());
    return reader;
}


// ---- Tests ------------------------------------------------------------------

TEST(FrameReaderTest, ReadsUbxFrame)
{
    // NAV-PVT: class=0x01, id=0x07, 92-byte payload
    auto raw = make_ubx(0x01, 0x07, std::vector<uint8_t>(92, 0));
    MockPort port(raw);
    auto reader = make_reader(port);

    auto frame = reader.read_frame(100);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->protocol, "UBX");
    EXPECT_EQ(frame->type,     "NAV-PVT");
    EXPECT_EQ(frame->raw,      raw);
}

TEST(FrameReaderTest, ReadsNmeaSentence)
{
    std::string sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    std::vector<uint8_t> raw(sentence.begin(), sentence.end());
    MockPort port(raw);
    auto reader = make_reader(port);

    auto frame = reader.read_frame(100);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->protocol, "NMEA");
    EXPECT_EQ(frame->type,     "GGA");
}

TEST(FrameReaderTest, ReadsRtcm3Frame)
{
    auto raw = make_rtcm3(1005, 19);  // standard MSG1005 length
    MockPort port(raw);
    auto reader = make_reader(port);

    auto frame = reader.read_frame(100);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->protocol, "RTCM3");
    EXPECT_EQ(frame->type,     "1005");
}

TEST(FrameReaderTest, SkipsGarbageBeforeUbx)
{
    // Garbage bytes followed by a valid NAV-SAT frame
    auto ubx = make_ubx(0x01, 0x35, std::vector<uint8_t>(8 + 12, 0));  // minimal NAV-SAT
    std::vector<uint8_t> raw = {0xAA, 0xBB, 0xCC};
    raw.insert(raw.end(), ubx.begin(), ubx.end());
    MockPort port(raw);
    auto reader = make_reader(port);

    auto frame = reader.read_frame(100);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->protocol, "UBX");
    EXPECT_EQ(frame->type,     "NAV-SAT");
}

TEST(FrameReaderTest, ReadsTwoConsecutiveFrames)
{
    auto ubx1 = make_ubx(0x05, 0x01, std::vector<uint8_t>(2, 0));  // ACK-ACK
    auto ubx2 = make_ubx(0x05, 0x00, std::vector<uint8_t>(2, 0));  // ACK-NAK
    std::vector<uint8_t> raw = ubx1;
    raw.insert(raw.end(), ubx2.begin(), ubx2.end());
    MockPort port(raw);
    auto reader = make_reader(port);

    auto f1 = reader.read_frame(100);
    ASSERT_TRUE(f1.has_value());
    EXPECT_EQ(f1->type, "ACK-ACK");

    auto f2 = reader.read_frame(100);
    ASSERT_TRUE(f2.has_value());
    EXPECT_EQ(f2->type, "ACK-NAK");
}

TEST(FrameReaderTest, TimeoutOnEmptyPort)
{
    MockPort port({});
    auto reader = make_reader(port);

    auto frame = reader.read_frame(10);
    EXPECT_FALSE(frame.has_value());
}

TEST(FrameReaderTest, ThrowsOnUbxChecksumError)
{
    auto raw = make_ubx(0x01, 0x07, std::vector<uint8_t>(92, 0));
    raw.back() ^= 0xFF;  // corrupt checksum
    MockPort port(raw);
    auto reader = make_reader(port);

    EXPECT_THROW(reader.read_frame(100), gnsspp::ParseError);
}

TEST(FrameReaderTest, ThrowsOnRtcm3CrcError)
{
    auto raw = make_rtcm3(1005, 19);
    raw.back() ^= 0xFF;  // corrupt CRC
    MockPort port(raw);
    auto reader = make_reader(port);

    EXPECT_THROW(reader.read_frame(100), gnsspp::ParseError);
}

TEST(FrameReaderTest, ReadsRtcm3Msm4)
{
    auto raw = make_rtcm3(1074, 64);
    MockPort port(raw);
    auto reader = make_reader(port);

    auto frame = reader.read_frame(100);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->protocol, "RTCM3");
    EXPECT_EQ(frame->type,     "1074");
}

TEST(FrameReaderTest, MixedUbxAndRtcm3)
{
    auto ubx  = make_ubx(0x05, 0x01, std::vector<uint8_t>(2, 0));  // ACK-ACK
    auto rtcm = make_rtcm3(1005, 19);
    std::vector<uint8_t> raw = ubx;
    raw.insert(raw.end(), rtcm.begin(), rtcm.end());
    MockPort port(raw);
    auto reader = make_reader(port);

    auto f1 = reader.read_frame(100);
    ASSERT_TRUE(f1.has_value());
    EXPECT_EQ(f1->protocol, "UBX");

    auto f2 = reader.read_frame(100);
    ASSERT_TRUE(f2.has_value());
    EXPECT_EQ(f2->protocol, "RTCM3");
    EXPECT_EQ(f2->type,     "1005");
}
