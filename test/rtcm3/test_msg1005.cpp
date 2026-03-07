#include <gtest/gtest.h>
#include <cmath>

#include "gnsspp/parsers/rtcm3/msg1005.hpp"
#include "rtcm3/bit_writer.hpp"


// Build a valid 1005 payload with controlled field values.
// ECEF coordinates are given in metres; internally scale by 1/0.0001 = 10000.
static std::vector<uint8_t> make_msg1005(uint16_t station_id,
                                         bool     gps,
                                         bool     glonass,
                                         bool     galileo,
                                         double   ecef_x_m,
                                         double   ecef_y_m,
                                         double   ecef_z_m)
{
    BitWriter bw;
    bw.write_u(12, 1005);           // DF002 message number
    bw.write_u(12, station_id);     // DF003
    bw.write_u(6,  0);              // DF021 ITRF year (irrelevant)
    bw.write_u(1,  gps ? 1 : 0);   // DF022
    bw.write_u(1,  glonass ? 1:0); // DF023
    bw.write_u(1,  galileo ? 1:0); // DF024
    bw.write_u(1,  0);             // DF141 ref-station indicator
    bw.write_s(38, static_cast<int64_t>(std::round(ecef_x_m / 0.0001)));  // DF025
    bw.write_u(1,  0);             // DF142
    bw.write_u(1,  0);             // DF001 reserved
    bw.write_s(38, static_cast<int64_t>(std::round(ecef_y_m / 0.0001)));  // DF026
    bw.write_u(2,  0);             // DF364
    bw.write_s(38, static_cast<int64_t>(std::round(ecef_z_m / 0.0001)));  // DF027
    return bw.data();
}


TEST(Msg1005Test, DecodeFields)
{
    // Kyiv approximate ECEF [m]
    auto payload = make_msg1005(42, true, true, false,
                                3595131.0, 2748700.0, 4881782.0);

    auto msg = gnsspp::decode_msg1005(payload);

    EXPECT_EQ(msg.station_id, 42);
    EXPECT_TRUE(msg.gps);
    EXPECT_TRUE(msg.glonass);
    EXPECT_FALSE(msg.galileo);
    EXPECT_NEAR(msg.ecef_x, 3595131.0, 0.0001);
    EXPECT_NEAR(msg.ecef_y, 2748700.0, 0.0001);
    EXPECT_NEAR(msg.ecef_z, 4881782.0, 0.0001);
}

TEST(Msg1005Test, NegativeCoordinates)
{
    auto payload = make_msg1005(1, true, false, false,
                                -2700000.0, -4200000.0, 3900000.0);
    auto msg = gnsspp::decode_msg1005(payload);
    EXPECT_NEAR(msg.ecef_x, -2700000.0, 0.0001);
    EXPECT_NEAR(msg.ecef_y, -4200000.0, 0.0001);
    EXPECT_NEAR(msg.ecef_z,  3900000.0, 0.0001);
}

TEST(Msg1005Test, TooShort)
{
    std::vector<uint8_t> short_payload(10, 0x00);
    EXPECT_THROW(gnsspp::decode_msg1005(short_payload), std::runtime_error);
}
