#include <gtest/gtest.h>
#include "gnsspp/parsers/ubx/rxm_rtcm.hpp"


class RxmRtcmTest : public ::testing::Test {
protected:
    // RxmRtcmRaw (8 bytes, version 2):
    //   version(1) + flags(1) + reserved0(2) + ref_station(2) + msg_type(2)
    std::vector<uint8_t> payload = {
        0x02,       // version = 2
        0x00,       // flags: bit0=crcFailed=0
        0x00, 0x00, // reserved0
        0x01, 0x00, // ref_station = 1
        0x32, 0x04, // msg_type = 1074 (0x0432)
    };
};


TEST_F(RxmRtcmTest, Decode) {
    auto msg = gnsspp::decode_rxm_rtcm(payload);

    EXPECT_EQ(msg.msg_type,    1074);
    EXPECT_EQ(msg.ref_station, 1);
    EXPECT_FALSE(msg.crc_failed);
}

TEST_F(RxmRtcmTest, DecodeCrcFailed) {
    std::vector<uint8_t> bad_crc_payload = {
        0x02,       // version = 2
        0x01,       // flags: bit0=crcFailed=1
        0x00, 0x00, // reserved0
        0x07, 0x00, // ref_station = 7
        0x11, 0x04, // msg_type = 1041 (0x0411)
    };
    auto msg = gnsspp::decode_rxm_rtcm(bad_crc_payload);
    EXPECT_TRUE(msg.crc_failed);
    EXPECT_EQ(msg.ref_station, 7);
}

TEST_F(RxmRtcmTest, TooShort) {
    std::vector<uint8_t> short_payload = { 0x02, 0x00 };
    EXPECT_THROW(gnsspp::decode_rxm_rtcm(short_payload), std::runtime_error);
}
