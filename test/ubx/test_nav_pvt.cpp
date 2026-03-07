#include <gtest/gtest.h>
#include "gnsspp/parsers/ubx/nav_pvt.hpp"


class NavPvtTest : public ::testing::Test {
protected:
    std::vector<uint8_t> payload = {
        // [0..3]   iTOW = 100000 ms
        0xA0, 0x86, 0x01, 0x00,
        // [4..5]   year = 2024
        0xE8, 0x07,
        // [6]      month = 3
        0x03,
        // [7]      day = 15
        0x0F,
        // [8]      hour = 12
        0x0C,
        // [9]      min = 30
        0x1E,
        // [10]     second = 45
        0x2D,
        // [11]     valid = 0x07
        0x07,
        // [12..15] tAcc = 100 ns
        0x64, 0x00, 0x00, 0x00,
        // [16..19] nano = 0
        0x00, 0x00, 0x00, 0x00,
        // [20]     fixType = 3 (3D)
        0x03,
        // [21]     flags = 0x01 (gnssFixOk=1, carrSoln=0)
        0x01,
        // [22]     flags2
        0x00,
        // [23]     numSV = 8
        0x08,
        // [24..27] lon = 30.5° → 305000000 = 0x122DEE40
        0x40, 0xEE, 0x2D, 0x12,
        // [28..31] lat = 50.45° → 504500000 = 0x1E120F20
        0x20, 0x0F, 0x12, 0x1E,
        // [32..35] height = 200000 mm
        0x40, 0x0D, 0x03, 0x00,
        // [36..39] hMSL = 195000 mm
        0xB8, 0xF9, 0x02, 0x00,
        // [40..43] hAcc = 2000 mm
        0xD0, 0x07, 0x00, 0x00,
        // [44..47] vAcc = 3000 mm
        0xB8, 0x0B, 0x00, 0x00,
        // [48..63] velN, velE, velD, gSpeed = 0
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        // [64..67] headMot = 0
        0x00, 0x00, 0x00, 0x00,
        // [68..71] sAcc = 1000 mm/s
        0xE8, 0x03, 0x00, 0x00,
        // [72..75] headAcc = 0
        0x00, 0x00, 0x00, 0x00,
        // [76..77] pDOP = 150 raw = 1.50
        0x96, 0x00,
        // [78..91] flags3, reserved0, headVeh, magDec, magAcc = 0
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
    };
};


TEST_F(NavPvtTest, Decode) {
    auto msg = gnsspp::decode_nav_pvt(payload);

    EXPECT_EQ(msg.itow,       100000u);
    EXPECT_EQ(msg.year,       2024);
    EXPECT_EQ(msg.month,      3);
    EXPECT_EQ(msg.day,        15);
    EXPECT_EQ(msg.hour,       12);
    EXPECT_EQ(msg.min,        30);
    EXPECT_EQ(msg.second,     45);
    EXPECT_EQ(msg.fix_type,   3);
    EXPECT_EQ(msg.num_sv,     8);
    EXPECT_TRUE(msg.gnss_fix_ok);
    EXPECT_EQ(msg.carr_soln,  0);
    EXPECT_NEAR(msg.lon,      30.5,   1e-6);
    EXPECT_NEAR(msg.lat,      50.45,  1e-6);
    EXPECT_EQ(msg.height,     200000);
    EXPECT_EQ(msg.h_msl,      195000);
    EXPECT_EQ(msg.h_acc,      2000u);
    EXPECT_EQ(msg.v_acc,      3000u);
    EXPECT_EQ(msg.s_acc,      1000u);
    EXPECT_NEAR(msg.p_dop,    1.50,   1e-3);
}

TEST_F(NavPvtTest, TooShort) {
    std::vector<uint8_t> short_payload = { 0xA0, 0x86, 0x01, 0x00 };
    EXPECT_THROW(gnsspp::decode_nav_pvt(short_payload), std::runtime_error);
}
