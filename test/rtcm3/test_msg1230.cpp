#include <gtest/gtest.h>

#include "gnsspp/rtcm3/msg1230.hpp"
#include "rtcm3/bit_writer.hpp"


static std::vector<uint8_t> make_msg1230(uint16_t station_id,
                                         bool     bias_ind,
                                         uint8_t  sig_mask,   // 4-bit: L1CA, L1P, L2CA, L2P
                                         int16_t  l1ca_raw,
                                         int16_t  l1p_raw,
                                         int16_t  l2ca_raw,
                                         int16_t  l2p_raw)
{
    BitWriter bw;
    bw.write_u(12, 1230);
    bw.write_u(12, station_id);
    bw.write_u(1,  bias_ind ? 1 : 0);  // DF421
    bw.write_u(3,  0);                  // reserved
    bw.write_u(4,  sig_mask);           // DF422
    if ((sig_mask >> 3) & 1) bw.write_s(16, l1ca_raw);
    if ((sig_mask >> 2) & 1) bw.write_s(16, l1p_raw);
    if ((sig_mask >> 1) & 1) bw.write_s(16, l2ca_raw);
    if ((sig_mask >> 0) & 1) bw.write_s(16, l2p_raw);
    return bw.data();
}


TEST(Msg1230Test, AllBiasesPresent)
{
    // scale: 0.02 m/lsb → 50 lsb = 1.00 m, -25 lsb = -0.50 m, 100 = 2.00 m, -200 = -4.00 m
    auto payload = make_msg1230(11, true, 0b1111, 50, -25, 100, -200);
    auto msg = gnsspp::decode_msg1230(payload);

    EXPECT_EQ(msg.station_id, 11);
    EXPECT_TRUE(msg.bias_indicator);

    EXPECT_TRUE(msg.l1ca.valid);
    EXPECT_NEAR(msg.l1ca.value_m,  1.00, 1e-9);

    EXPECT_TRUE(msg.l1p.valid);
    EXPECT_NEAR(msg.l1p.value_m,  -0.50, 1e-9);

    EXPECT_TRUE(msg.l2ca.valid);
    EXPECT_NEAR(msg.l2ca.value_m,  2.00, 1e-9);

    EXPECT_TRUE(msg.l2p.valid);
    EXPECT_NEAR(msg.l2p.value_m,  -4.00, 1e-9);
}

TEST(Msg1230Test, SomeFieldsMissing)
{
    // Only L1 C/A and L2 P present (mask = 0b1001)
    auto payload = make_msg1230(2, true, 0b1001, 10, 0, 0, -5);
    auto msg = gnsspp::decode_msg1230(payload);

    EXPECT_TRUE (msg.l1ca.valid);
    EXPECT_FALSE(msg.l1p.valid);
    EXPECT_FALSE(msg.l2ca.valid);
    EXPECT_TRUE (msg.l2p.valid);
    EXPECT_NEAR(msg.l1ca.value_m,  0.20, 1e-9);
    EXPECT_NEAR(msg.l2p.value_m,  -0.10, 1e-9);
}

TEST(Msg1230Test, NoBiases)
{
    // Signal mask = 0 → no bias fields, bias_indicator = false
    auto payload = make_msg1230(0, false, 0b0000, 0, 0, 0, 0);
    auto msg = gnsspp::decode_msg1230(payload);

    EXPECT_FALSE(msg.bias_indicator);
    EXPECT_FALSE(msg.l1ca.valid);
    EXPECT_FALSE(msg.l1p.valid);
    EXPECT_FALSE(msg.l2ca.valid);
    EXPECT_FALSE(msg.l2p.valid);
}

TEST(Msg1230Test, TooShort)
{
    std::vector<uint8_t> short_payload(2, 0x00);
    EXPECT_THROW(gnsspp::decode_msg1230(short_payload), std::runtime_error);
}
