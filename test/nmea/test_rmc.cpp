#include <gtest/gtest.h>
#include "gnsspp/parsers/nmea/rmc.hpp"


class RMCTest : public ::testing::Test {
protected:
    // $GNRMC,HHMMSS.ss,status,DDMM.MMMM,N/S,DDDMM.MMMM,E/W,speed,course,DDMMYY,...*cs
    const std::string payload =
        "$GNRMC,123519.00,A,4807.0380,N,01131.0000,E,1.5,270.0,150324,,,A*00\r\n";
};


TEST_F(RMCTest, Decode) {
    auto msg = gnsspp::decode_rmc(payload);

    EXPECT_EQ(msg.utc,   "123519.00");
    EXPECT_TRUE(msg.active);
    EXPECT_NEAR(msg.lat,          48.1173, 1e-4); // 4807.0380 N
    EXPECT_NEAR(msg.lon,          11.5167, 1e-4); // 01131.0000 E
    EXPECT_NEAR(msg.speed_knots,  1.5f,    0.01f);
    EXPECT_NEAR(msg.course_deg,   270.0f,  0.01f);
    EXPECT_EQ(msg.date,  "150324");
}

TEST_F(RMCTest, WarningStatus) {
    // status='V' means data invalid (dead reckoning or no fix)
    const std::string warn =
        "$GNRMC,000000.00,V,0000.0000,N,00000.0000,E,0.0,,010100,,,N*00\r\n";
    auto msg = gnsspp::decode_rmc(warn);
    EXPECT_FALSE(msg.active);
}

TEST_F(RMCTest, TooFewFields) {
    EXPECT_THROW(gnsspp::decode_rmc("$GNRMC,123519*00\r\n"), std::runtime_error);
}
