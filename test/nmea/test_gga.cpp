#include <gtest/gtest.h>
#include "gnsspp/nmea/gga.hpp"


class GGATest : public ::testing::Test {
protected:
    // $GNGGA,HHMMSS.ss,DDMM.MMMM,N/S,DDDMM.MMMM,E/W,quality,numSV,hdop,alt,M,geoid,M,,*cs
    const std::string payload =
        "$GNGGA,123519.00,4807.0380,N,01131.0000,E,1,08,0.90,545.4,M,46.9,M,,*6A\r\n";
};


TEST_F(GGATest, Decode) {
    auto msg = gnsspp::decode_gga(payload);

    EXPECT_EQ(msg.utc,          "123519.00");
    EXPECT_NEAR(msg.lat,         48.1173,  1e-4); // 4807.0380 N → 48 + 7.038/60
    EXPECT_NEAR(msg.lon,         11.5167,  1e-4); // 01131.0000 E → 11 + 31/60
    EXPECT_EQ(msg.fix_quality,   1);
    EXPECT_EQ(msg.num_sv,        8);
    EXPECT_NEAR(msg.hdop,        0.90f,   0.01f);
    EXPECT_NEAR(msg.alt_m,       545.4f,   0.1f);
    EXPECT_NEAR(msg.geoid_sep_m, 46.9f,    0.1f);
}

TEST_F(GGATest, SouthWest) {
    // Southern hemisphere, western longitude
    const std::string sw =
        "$GPGGA,010203.00,3317.5400,S,07040.2000,W,4,12,0.60,3200.0,M,-30.0,M,,*00\r\n";
    auto msg = gnsspp::decode_gga(sw);

    EXPECT_NEAR(msg.lat, -(33.0 + 17.54/60.0), 1e-4);
    EXPECT_NEAR(msg.lon, -(70.0 + 40.20/60.0), 1e-4);
    EXPECT_EQ(msg.fix_quality, 4); // RTK fixed
}

TEST_F(GGATest, TooFewFields) {
    EXPECT_THROW(gnsspp::decode_gga("$GNGGA,123519.00,4807.0380,N,01131"), std::runtime_error);
}
