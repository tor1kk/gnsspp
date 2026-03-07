#include <gtest/gtest.h>
#include "gnsspp/ubx/nav_sat.hpp"


class NavSatTest : public ::testing::Test {
protected:
    std::vector<uint8_t> payload = {
        // --- NavSatHeader (8 bytes) ---
        // [0..3]  iTOW = 100000 ms
        0xA0, 0x86, 0x01, 0x00,
        // [4]     version = 1
        0x01,
        // [5]     numSvs = 2
        0x02,
        // [6..7]  reserved
        0x00, 0x00,

        // --- SvInfoRaw[0]: GPS sv_id=5, cno=35, elev=45, azim=180, prRes=1.5m ---
        0x00,       // gnssId = 0 (GPS)
        0x05,       // svId = 5
        0x23,       // cno = 35 dBHz
        0x2D,       // elev = 45 deg
        0xB4, 0x00, // azim = 180 deg
        0x0F, 0x00, // prResRaw = 15 (1.5 m × 10)
        // flags = 0x0000091F: qualityInd=7, svUsed=1, health=1, orbitSource=1, ephAvail=1
        0x1F, 0x09, 0x00, 0x00,

        // --- SvInfoRaw[1]: GLONASS sv_id=10, cno=28, elev=20, azim=90, prRes=-0.5m ---
        0x06,       // gnssId = 6 (GLONASS)
        0x0A,       // svId = 10
        0x1C,       // cno = 28 dBHz
        0x14,       // elev = 20 deg
        0x5A, 0x00, // azim = 90 deg
        0xFB, 0xFF, // prResRaw = -5 (-0.5 m × 10)
        // flags = 0x00001A04: qualityInd=4, orbitSource=2, ephAvail=1, almAvail=1
        0x04, 0x1A, 0x00, 0x00,
    };
};


TEST_F(NavSatTest, Decode) {
    auto msg = gnsspp::decode_nav_sat(payload);

    EXPECT_EQ(msg.itow,    100000u);
    EXPECT_EQ(msg.num_svs, 2);
    ASSERT_EQ(msg.svs.size(), 2u);

    // SV0 — GPS
    EXPECT_EQ(msg.svs[0].gnss_id,      0);
    EXPECT_EQ(msg.svs[0].sv_id,        5);
    EXPECT_EQ(msg.svs[0].cno,          35);
    EXPECT_EQ(msg.svs[0].elev,         45);
    EXPECT_EQ(msg.svs[0].azim,         180);
    EXPECT_NEAR(msg.svs[0].pr_res,     1.5f,  0.01f);
    EXPECT_EQ(msg.svs[0].quality_ind,  7);
    EXPECT_TRUE(msg.svs[0].sv_used);
    EXPECT_EQ(msg.svs[0].health,       1);
    EXPECT_EQ(msg.svs[0].orbit_source, 1);
    EXPECT_TRUE(msg.svs[0].eph_avail);

    // SV1 — GLONASS
    EXPECT_EQ(msg.svs[1].gnss_id,      6);
    EXPECT_EQ(msg.svs[1].sv_id,        10);
    EXPECT_EQ(msg.svs[1].cno,          28);
    EXPECT_NEAR(msg.svs[1].pr_res,    -0.5f, 0.01f);
    EXPECT_EQ(msg.svs[1].quality_ind,  4);
    EXPECT_FALSE(msg.svs[1].sv_used);
    EXPECT_EQ(msg.svs[1].orbit_source, 2);
    EXPECT_TRUE(msg.svs[1].eph_avail);
    EXPECT_TRUE(msg.svs[1].alm_avail);
}

TEST_F(NavSatTest, TooShort) {
    std::vector<uint8_t> short_payload = { 0xA0, 0x86, 0x01 };
    EXPECT_THROW(gnsspp::decode_nav_sat(short_payload), std::runtime_error);
}
