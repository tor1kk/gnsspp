#include <gtest/gtest.h>
#include "gnsspp/parsers/nmea/gsa.hpp"


class GSATest : public ::testing::Test {
protected:
    // $GNGSA,mode,fixType,prn1..12,pdop,hdop,vdop[,systemId]*cs
    // 9 non-empty PRNs: 01,02,03,08,09,10,16,23,31 → sv_count=9
    const std::string payload =
        "$GNGSA,A,3,01,02,03,08,09,10,16,23,31,,,,1.23,0.65,1.04,1*0A\r\n";
};


TEST_F(GSATest, Decode) {
    auto msg = gnsspp::decode_gsa(payload);

    EXPECT_TRUE(msg.auto_mode);
    EXPECT_EQ(msg.fix_type, 3);
    EXPECT_EQ(msg.sv_count, 9);
    EXPECT_EQ(msg.sv_prns[0], 1);
    EXPECT_EQ(msg.sv_prns[1], 2);
    EXPECT_EQ(msg.sv_prns[8], 31);
    EXPECT_NEAR(msg.pdop, 1.23f, 0.01f);
    EXPECT_NEAR(msg.hdop, 0.65f, 0.01f);
    EXPECT_NEAR(msg.vdop, 1.04f, 0.01f);
    EXPECT_EQ(msg.system_id, 1);
}

TEST_F(GSATest, ManualMode2D) {
    const std::string s =
        "$GPGSA,M,2,05,08,,,,,,,,,,,2.10,1.80,1.10*00\r\n";
    auto msg = gnsspp::decode_gsa(s);
    EXPECT_FALSE(msg.auto_mode);
    EXPECT_EQ(msg.fix_type, 2);
    EXPECT_EQ(msg.sv_count, 2);
}

TEST_F(GSATest, TooFewFields) {
    EXPECT_THROW(gnsspp::decode_gsa("$GNGSA,A*00\r\n"), std::runtime_error);
}
