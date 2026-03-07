#include <gtest/gtest.h>
#include "gnsspp/ubx/ack.hpp"
#include "gnsspp/error.hpp"


TEST(Ack, DecodeAckAck)
{
    std::vector<uint8_t> payload = {0x06, 0x8A};  // CFG-VALSET
    auto ack = gnsspp::decode_ack_ack(payload);
    EXPECT_EQ(ack.cls_id, 0x06);
    EXPECT_EQ(ack.msg_id, 0x8A);
}

TEST(Ack, DecodeAckNak)
{
    std::vector<uint8_t> payload = {0x06, 0x8A};
    auto nak = gnsspp::decode_ack_nak(payload);
    EXPECT_EQ(nak.cls_id, 0x06);
    EXPECT_EQ(nak.msg_id, 0x8A);
}

TEST(Ack, TooShortThrows)
{
    std::vector<uint8_t> short_payload = {0x06};
    EXPECT_THROW(gnsspp::decode_ack_ack(short_payload), gnsspp::ParseError);
    EXPECT_THROW(gnsspp::decode_ack_nak(short_payload), gnsspp::ParseError);
}
