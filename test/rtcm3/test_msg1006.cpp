#include <gtest/gtest.h>
#include <cmath>

#include "gnsspp/rtcm3/msg1006.hpp"
#include "rtcm3/bit_writer.hpp"


static std::vector<uint8_t> make_msg1006(uint16_t station_id,
                                         double   ecef_x_m,
                                         double   ecef_y_m,
                                         double   ecef_z_m,
                                         double   ant_height_m)
{
    BitWriter bw;
    bw.write_u(12, 1006);
    bw.write_u(12, station_id);
    bw.write_u(6,  0);          // ITRF year
    bw.write_u(1,  1);          // GPS
    bw.write_u(1,  0);          // GLONASS
    bw.write_u(1,  0);          // Galileo
    bw.write_u(1,  0);          // ref-station indicator
    bw.write_s(38, static_cast<int64_t>(std::round(ecef_x_m / 0.0001)));
    bw.write_u(1,  0);
    bw.write_u(1,  0);
    bw.write_s(38, static_cast<int64_t>(std::round(ecef_y_m / 0.0001)));
    bw.write_u(2,  0);
    bw.write_s(38, static_cast<int64_t>(std::round(ecef_z_m / 0.0001)));
    bw.write_u(16, static_cast<uint64_t>(std::round(ant_height_m / 0.0001))); // DF028
    return bw.data();
}


TEST(Msg1006Test, DecodeFields)
{
    auto payload = make_msg1006(7, 3595131.0, 2748700.0, 4881782.0, 1.5432);
    auto msg = gnsspp::decode_msg1006(payload);

    EXPECT_EQ(msg.station_id, 7);
    EXPECT_TRUE(msg.gps);
    EXPECT_NEAR(msg.ecef_x,         3595131.0, 0.0001);
    EXPECT_NEAR(msg.ecef_y,         2748700.0, 0.0001);
    EXPECT_NEAR(msg.ecef_z,         4881782.0, 0.0001);
    EXPECT_NEAR(msg.antenna_height, 1.5432,    0.0001);
}

TEST(Msg1006Test, ZeroHeight)
{
    auto payload = make_msg1006(0, 0.0, 0.0, 0.0, 0.0);
    auto msg = gnsspp::decode_msg1006(payload);
    EXPECT_NEAR(msg.antenna_height, 0.0, 1e-9);
}

TEST(Msg1006Test, TooShort)
{
    std::vector<uint8_t> short_payload(18, 0x00);  // one byte short of 19 (1005 size)
    EXPECT_THROW(gnsspp::decode_msg1006(short_payload), std::runtime_error);
}
