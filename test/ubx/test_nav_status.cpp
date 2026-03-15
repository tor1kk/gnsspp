#include <gtest/gtest.h>
#include "gnsspp/ubx/nav_status.hpp"

// NAV-STATUS payload layout (16 bytes, little-endian):
//  [0..3]  iTOW    = 100000 ms
//  [4]     gpsFix  = 3 (3D)
//  [5]     flags   = 0x03 (gpsFixOk=1, diffSoln=1)
//  [6]     fixStat = 0x0C (carrSolnValid=1, carrSoln=1 → float RTK)
//  [7]     flags2  = 0
//  [8..11] ttff    = 5000 ms
//  [12..15] msss   = 60000 ms

static std::vector<uint8_t> make_payload(
    uint8_t gps_fix = 3,
    uint8_t flags   = 0x03,
    uint8_t fix_stat = 0x0C)
{
    std::vector<uint8_t> p(16, 0);
    // iTOW = 100000 = 0x000186A0
    p[0] = 0xA0; p[1] = 0x86; p[2] = 0x01; p[3] = 0x00;
    p[4] = gps_fix;
    p[5] = flags;
    p[6] = fix_stat;
    p[7] = 0x00;
    // ttff = 5000 = 0x00001388
    p[8] = 0x88; p[9] = 0x13; p[10] = 0x00; p[11] = 0x00;
    // msss = 60000 = 0x0000EA60
    p[12] = 0x60; p[13] = 0xEA; p[14] = 0x00; p[15] = 0x00;
    return p;
}

TEST(NavStatusTest, Decode3DFloatRTK)
{
    auto msg = gnsspp::decode_nav_status(make_payload());

    EXPECT_EQ(msg.itow,            100000u);
    EXPECT_EQ(msg.gps_fix,         3);
    EXPECT_TRUE(msg.gps_fix_ok);
    EXPECT_TRUE(msg.diff_soln);
    EXPECT_TRUE(msg.carr_soln_valid);
    EXPECT_EQ(msg.carr_soln,       1);   // float RTK
    EXPECT_EQ(msg.ttff,            5000u);
    EXPECT_EQ(msg.msss,            60000u);
}

TEST(NavStatusTest, NoFix)
{
    auto msg = gnsspp::decode_nav_status(make_payload(0, 0x00, 0x00));
    EXPECT_EQ(msg.gps_fix, 0);
    EXPECT_FALSE(msg.gps_fix_ok);
    EXPECT_FALSE(msg.diff_soln);
    EXPECT_FALSE(msg.carr_soln_valid);
    EXPECT_EQ(msg.carr_soln, 0);
}

TEST(NavStatusTest, FixedRTK)
{
    // carrSoln=2 (fixed) lives at fix_stat bits [4:3] => 0b00010100 = 0x14, carrSolnValid=bit2 set
    auto msg = gnsspp::decode_nav_status(make_payload(3, 0x01, 0x14));
    EXPECT_EQ(msg.carr_soln, 2);
    EXPECT_TRUE(msg.carr_soln_valid);
}

TEST(NavStatusTest, TooShort)
{
    std::vector<uint8_t> p(15, 0);
    EXPECT_THROW(gnsspp::decode_nav_status(p), std::runtime_error);
}
