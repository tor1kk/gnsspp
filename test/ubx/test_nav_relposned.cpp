#include <gtest/gtest.h>
#include "gnsspp/ubx/nav_relposned.hpp"


class NavRelPosNedTest : public ::testing::Test {
protected:
    // NavRelPosNedRaw (64 bytes, version 1):
    //   version(1) + reserved0(1) + ref_station_id(2) + itow(4)
    //   + rel_pos_n(4) + rel_pos_e(4) + rel_pos_d(4) + rel_pos_length(4)
    //   + rel_pos_heading(4) + reserved1(4)
    //   + hp_n(1) + hp_e(1) + hp_d(1) + hp_length(1)
    //   + acc_n(4) + acc_e(4) + acc_d(4) + acc_length(4) + acc_heading(4)
    //   + reserved2(4) + flags(4)
    std::vector<uint8_t> payload = {
        0x01,                   // version = 1
        0x00,                   // reserved0
        0x05, 0x00,             // ref_station_id = 5
        0xE0, 0x93, 0x04, 0x00, // itow = 300000 ms
        0x96, 0x00, 0x00, 0x00, // rel_pos_n = 150 cm  → +1.50 m
        0xB5, 0xFF, 0xFF, 0xFF, // rel_pos_e = -75 cm  → -0.75 m  (int32 LE)
        0x14, 0x00, 0x00, 0x00, // rel_pos_d = 20 cm   → +0.20 m
        0xAA, 0x00, 0x00, 0x00, // rel_pos_length = 170 cm → 1.70 m
        0x00, 0x00, 0x00, 0x00, // rel_pos_heading = 0 deg
        0x00, 0x00, 0x00, 0x00, // reserved1
        0x04,                   // rel_pos_hp_n = +4 (0.1mm) → +0.0004 m
        0xFE,                   // rel_pos_hp_e = -2         → -0.0002 m
        0x03,                   // rel_pos_hp_d = +3         → +0.0003 m
        0x05,                   // rel_pos_hp_length = +5    → +0.0005 m
        0xF4, 0x01, 0x00, 0x00, // acc_n = 500 (0.1mm) → 0.05 m
        0x20, 0x03, 0x00, 0x00, // acc_e = 800         → 0.08 m
        0xE8, 0x03, 0x00, 0x00, // acc_d = 1000        → 0.10 m
        0x84, 0x03, 0x00, 0x00, // acc_length = 900    → 0.09 m
        0x00, 0x00, 0x00, 0x00, // acc_heading = 0 deg
        0x00, 0x00, 0x00, 0x00, // reserved2
        // flags = 0x17 = 0b00010111:
        //   bit0=1: gnss_fix_ok, bit1=1: diff_soln, bit2=1: rel_pos_valid
        //   bits3-4=10: carr_soln=2 (fixed RTK)
        0x17, 0x00, 0x00, 0x00,
    };
};


TEST_F(NavRelPosNedTest, Decode) {
    auto msg = gnsspp::decode_nav_relposned(payload);

    EXPECT_EQ(msg.itow,           300000u);
    EXPECT_EQ(msg.ref_station_id, 5);
    EXPECT_NEAR(msg.rel_pos_n_m,      1.5004,  1e-6);
    EXPECT_NEAR(msg.rel_pos_e_m,     -0.7502,  1e-6);
    EXPECT_NEAR(msg.rel_pos_d_m,      0.2003,  1e-6);
    EXPECT_NEAR(msg.rel_pos_length_m, 1.7005,  1e-6);
    EXPECT_NEAR(msg.rel_pos_heading_deg, 0.0,  1e-3);
    EXPECT_NEAR(msg.acc_n_m,     0.05f,  1e-4f);
    EXPECT_NEAR(msg.acc_e_m,     0.08f,  1e-4f);
    EXPECT_NEAR(msg.acc_d_m,     0.10f,  1e-4f);
    EXPECT_NEAR(msg.acc_length_m, 0.09f, 1e-4f);
    EXPECT_TRUE(msg.gnss_fix_ok);
    EXPECT_TRUE(msg.diff_soln);
    EXPECT_TRUE(msg.rel_pos_valid);
    EXPECT_EQ(msg.carr_soln, 2);
    EXPECT_FALSE(msg.ref_pos_miss);
    EXPECT_FALSE(msg.ref_obs_miss);
    EXPECT_FALSE(msg.rel_pos_heading_valid);
}

TEST_F(NavRelPosNedTest, TooShort) {
    std::vector<uint8_t> short_payload = { 0x01, 0x00, 0x05 };
    EXPECT_THROW(gnsspp::decode_nav_relposned(short_payload), std::runtime_error);
}
