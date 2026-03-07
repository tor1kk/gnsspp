#include <gtest/gtest.h>
#include "gnsspp/parsers/nmea/gsv.hpp"


class GSVTest : public ::testing::Test {
protected:
    // Message 1 of 3: 4 satellites + signal_id=1 at end (NMEA 4.11)
    // Fields: $GPGSV,totalMsgs,msgNum,totalSvs,prn,elev,azim,snr,...,signalId*cs
    const std::string payload =
        "$GPGSV,3,1,11,01,67,275,41,02,57,327,39,03,40,259,36,08,48,092,39,1*67\r\n";
};


TEST_F(GSVTest, Decode) {
    auto msg = gnsspp::decode_gsv(payload);

    EXPECT_EQ(msg.total_msgs, 3);
    EXPECT_EQ(msg.msg_num,    1);
    EXPECT_EQ(msg.total_svs,  11);
    EXPECT_EQ(msg.signal_id,  1);
    ASSERT_EQ(msg.satellites.size(), 4u);

    EXPECT_EQ(msg.satellites[0].prn,  1);
    EXPECT_EQ(msg.satellites[0].elev, 67);
    EXPECT_EQ(msg.satellites[0].azim, 275);
    EXPECT_EQ(msg.satellites[0].snr,  41);

    EXPECT_EQ(msg.satellites[3].prn,  8);
    EXPECT_EQ(msg.satellites[3].elev, 48);
    EXPECT_EQ(msg.satellites[3].azim, 92);
    EXPECT_EQ(msg.satellites[3].snr,  39);
}

TEST_F(GSVTest, NoSignalId) {
    // Older NMEA 4.10 sentences omit signal_id — signal_id should be 0
    const std::string s =
        "$GPGSV,1,1,02,05,30,100,35,10,55,200,40*00\r\n";
    auto msg = gnsspp::decode_gsv(s);
    EXPECT_EQ(msg.total_msgs, 1);
    EXPECT_EQ(msg.total_svs,  2);
    EXPECT_EQ(msg.signal_id,  0);
    EXPECT_EQ(msg.satellites.size(), 2u);
}

TEST_F(GSVTest, TooFewFields) {
    EXPECT_THROW(gnsspp::decode_gsv("$GPGSV*00\r\n"), std::runtime_error);
}
