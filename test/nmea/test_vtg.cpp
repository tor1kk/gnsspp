#include <gtest/gtest.h>
#include "gnsspp/parsers/nmea/vtg.hpp"


class VTGTest : public ::testing::Test {
protected:
    // $GNVTG,coursTrue,T,courseMag,M,speedN,N,speedK,K,mode*cs
    const std::string payload =
        "$GNVTG,270.5,T,265.0,M,1.5,N,2.78,K,A*00\r\n";
};


TEST_F(VTGTest, Decode) {
    auto msg = gnsspp::decode_vtg(payload);

    EXPECT_NEAR(msg.course_true_deg, 270.5f, 0.01f);
    EXPECT_NEAR(msg.course_mag_deg,  265.0f, 0.01f);
    EXPECT_NEAR(msg.speed_knots,     1.5f,   0.01f);
    EXPECT_NEAR(msg.speed_kmh,       2.78f,  0.01f);
}

TEST_F(VTGTest, Stationary) {
    // Speed = 0, no course
    const std::string s = "$GNVTG,,T,,M,0.0,N,0.0,K,N*00\r\n";
    auto msg = gnsspp::decode_vtg(s);
    EXPECT_NEAR(msg.speed_knots, 0.0f, 0.01f);
    EXPECT_NEAR(msg.speed_kmh,   0.0f, 0.01f);
}

TEST_F(VTGTest, TooFewFields) {
    EXPECT_THROW(gnsspp::decode_vtg("$GNVTG,270*00\r\n"), std::runtime_error);
}
