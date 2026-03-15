#include <gtest/gtest.h>
#include "gnsspp/ubx/nav_hpposllh.hpp"

// NAV-HPPOSLLH payload layout (36 bytes, little-endian):
//  [0]      version  = 0
//  [1..2]   reserved = 0
//  [3]      flags    (bit0 = invalidLlh)
//  [4..7]   iTOW     = 360000 ms
//  [8..11]  lon_raw  = 305000000 (30.5° * 1e7)
//  [12..15] lat_raw  = 504500000 (50.45° * 1e7)
//  [16..19] height_raw = 200000 mm
//  [20..23] h_msl_raw  = 195000 mm
//  [24]     lon_hp   = 50  (50 * 1e-9 deg)
//  [25]     lat_hp   = 20  (20 * 1e-9 deg)
//  [26]     height_hp = 5  (0.5 mm)
//  [27]     h_msl_hp  = 3  (0.3 mm)
//  [28..31] h_acc_raw = 15  (1.5 mm)
//  [32..35] v_acc_raw = 25  (2.5 mm)

static std::vector<uint8_t> make_payload()
{
    std::vector<uint8_t> p(36, 0);
    // flags = 0 (valid)
    p[3] = 0x00;
    // iTOW = 360000 = 0x00057E40
    p[4] = 0x40; p[5] = 0x7E; p[6] = 0x05; p[7] = 0x00;
    // lon_raw = 305000000 = 0x122DEE40
    p[8] = 0x40; p[9] = 0xEE; p[10] = 0x2D; p[11] = 0x12;
    // lat_raw = 504500000 = 0x1E120F20
    p[12] = 0x20; p[13] = 0x0F; p[14] = 0x12; p[15] = 0x1E;
    // height_raw = 200000 = 0x00030D40
    p[16] = 0x40; p[17] = 0x0D; p[18] = 0x03; p[19] = 0x00;
    // h_msl_raw = 195000 = 0x0002F9B8
    p[20] = 0xB8; p[21] = 0xF9; p[22] = 0x02; p[23] = 0x00;
    // lon_hp = 50, lat_hp = 20, height_hp = 5, h_msl_hp = 3
    p[24] = 50; p[25] = 20; p[26] = 5; p[27] = 3;
    // h_acc_raw = 15
    p[28] = 15; p[29] = 0; p[30] = 0; p[31] = 0;
    // v_acc_raw = 25
    p[32] = 25; p[33] = 0; p[34] = 0; p[35] = 0;
    return p;
}

TEST(NavHpPosLlhTest, Decode)
{
    auto msg = gnsspp::decode_nav_hpposllh(make_payload());

    EXPECT_EQ(msg.itow, 360000u);
    EXPECT_FALSE(msg.invalid_llh);
    // lon = 30.5 + 50e-9 = 30.50000005
    EXPECT_NEAR(msg.lon, 30.5 + 50e-9, 1e-9);
    // lat = 50.45 + 20e-9 = 50.45000002
    EXPECT_NEAR(msg.lat, 50.45 + 20e-9, 1e-9);
    // height = 200000 + 5 * 0.1 = 200000.5 mm
    EXPECT_NEAR(msg.height, 200000.5, 1e-6);
    // h_msl = 195000 + 3 * 0.1 = 195000.3 mm
    EXPECT_NEAR(msg.h_msl, 195000.3, 1e-6);
    // h_acc = 15 * 0.1 = 1.5 mm
    EXPECT_NEAR(msg.h_acc, 1.5, 1e-6);
    // v_acc = 25 * 0.1 = 2.5 mm
    EXPECT_NEAR(msg.v_acc, 2.5, 1e-6);
}

TEST(NavHpPosLlhTest, InvalidFlag)
{
    auto p = make_payload();
    p[3] = 0x01;  // set invalidLlh bit
    auto msg = gnsspp::decode_nav_hpposllh(p);
    EXPECT_TRUE(msg.invalid_llh);
}

TEST(NavHpPosLlhTest, TooShort)
{
    std::vector<uint8_t> p(35, 0);
    EXPECT_THROW(gnsspp::decode_nav_hpposllh(p), std::runtime_error);
}
