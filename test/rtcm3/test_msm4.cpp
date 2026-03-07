#include <gtest/gtest.h>
#include <cmath>

#include "gnsspp/parsers/rtcm3/msm4.hpp"
#include "rtcm3/bit_writer.hpp"


// Build a minimal GPS MSM4 (1074) payload:
//   - 1 satellite: PRN 1 (sat_mask bit 63)
//   - 1 signal:    GPS L1 C/A = signal 2 (sig_mask bit 30)
//   - 1 cell:      (PRN1, sig2) present
//   - Rough range: 75 ms integer + 512 * 2^-10 ms fractional = 75.5 ms
//   - Fine pseudorange: 0 (no correction)
//   - Fine phase range: 0
//   - Lock time: 5, half-cycle: 0, CNR: 40 dBHz
static std::vector<uint8_t> make_gps_msm4(uint16_t station_id, uint32_t epoch_ms)
{
    BitWriter bw;
    bw.write_u(12, 1074);              // DF002
    bw.write_u(12, station_id);        // DF003
    bw.write_u(30, epoch_ms);          // Epoch time
    bw.write_u(1,  0);                 // DF393 multiple message
    bw.write_u(3,  0);                 // DF409 IODS
    bw.write_u(7,  0);                 // Reserved
    bw.write_u(2,  0);                 // DF411 clock steering
    bw.write_u(2,  0);                 // DF412 external clock
    bw.write_u(1,  0);                 // DF417 smoothing
    bw.write_u(3,  0);                 // DF418 smoothing interval

    // DF394: satellite mask — PRN 1 = bit 63
    bw.write_u(64, uint64_t(1) << 63);

    // DF395: signal mask — signal 2 = bit 30 (MSB = signal 1 = bit 31)
    bw.write_u(32, uint32_t(1) << 30);

    // DF396: cell mask — 1 sat × 1 sig = 1 bit, cell present
    bw.write_u(1, 1);

    // Satellite data: DF397 (integer ms) + DF398 (frac ms)
    bw.write_u(8,  75);   // DF397: 75 ms
    bw.write_u(10, 512);  // DF398: 512 * 2^-10 = 0.5 ms  → rough = 75.5 ms

    // Signal data (1 cell)
    bw.write_s(15, 0);    // DF400 fine pseudorange: 0
    bw.write_s(22, 0);    // DF401 fine phase range: 0
    bw.write_u(4,  5);    // DF402 lock time: 5
    bw.write_u(1,  0);    // DF420 half-cycle: 0
    bw.write_u(6,  40);   // DF403 CNR: 40 dBHz

    return bw.data();
}


TEST(Msm4Test, ParseGpsMsm4)
{
    auto payload = make_gps_msm4(3, 345600000u);  // epoch ~96h into week
    auto msg = gnsspp::decode_msm4(payload);

    EXPECT_EQ(msg.msg_type,      1074);
    EXPECT_EQ(msg.station_id,    3);
    EXPECT_EQ(msg.epoch_time_ms, 345600000u);
    EXPECT_FALSE(msg.multiple_msg);

    ASSERT_EQ(msg.satellites.size(), 1u);
    EXPECT_EQ(msg.satellites[0].id, 1);
    EXPECT_NEAR(msg.satellites[0].rough_range_ms, 75.5, 1e-6);

    ASSERT_EQ(msg.signals.size(), 1u);
    EXPECT_EQ(msg.signals[0].sat_id, 1);
    EXPECT_EQ(msg.signals[0].sig_id, 2);
    // pseudorange = 75.5 ms * 299792.458 m/ms
    EXPECT_NEAR(msg.signals[0].pseudorange_m, 75.5 * 299792.458, 1.0);
    EXPECT_NEAR(msg.signals[0].phase_range_m, 75.5 * 299792.458, 1.0);
    EXPECT_EQ(msg.signals[0].lock_time_ind, 5);
    EXPECT_FALSE(msg.signals[0].half_cycle);
    EXPECT_FLOAT_EQ(msg.signals[0].cnr_dbhz, 40.0f);
}

TEST(Msm4Test, InvalidSatelliteSlot)
{
    // DF397 = 0xFF marks invalid satellite slot → rough_range_ms must be NaN
    BitWriter bw;
    bw.write_u(12, 1074);
    bw.write_u(12, 0);
    bw.write_u(30, 0);
    bw.write_u(1,  0); bw.write_u(3, 0); bw.write_u(7, 0);
    bw.write_u(2,  0); bw.write_u(2, 0); bw.write_u(1, 0); bw.write_u(3, 0);
    bw.write_u(64, uint64_t(1) << 63);
    bw.write_u(32, uint32_t(1) << 30);
    bw.write_u(1,  1);          // cell present
    bw.write_u(8,  0xFF);       // DF397 = invalid
    bw.write_u(10, 0);
    bw.write_s(15, 0);
    bw.write_s(22, 0);
    bw.write_u(4,  0); bw.write_u(1, 0); bw.write_u(6, 0);

    auto msg = gnsspp::decode_msm4(bw.data());
    EXPECT_TRUE(std::isnan(msg.satellites[0].rough_range_ms));
    EXPECT_TRUE(std::isnan(msg.signals[0].pseudorange_m));
    EXPECT_TRUE(std::isnan(msg.signals[0].phase_range_m));
}

TEST(Msm4Test, MultipleSatsAndSignals)
{
    // 2 satellites (PRN 1 and PRN 3), 1 signal, both cells present
    BitWriter bw;
    bw.write_u(12, 1074);
    bw.write_u(12, 5);
    bw.write_u(30, 1000u);
    bw.write_u(1,  0); bw.write_u(3, 0); bw.write_u(7, 0);
    bw.write_u(2,  0); bw.write_u(2, 0); bw.write_u(1, 0); bw.write_u(3, 0);
    // Satellite mask: PRN 1 (bit 63) + PRN 3 (bit 61)
    bw.write_u(64, (uint64_t(1) << 63) | (uint64_t(1) << 61));
    // Signal mask: signal 2 (bit 30)
    bw.write_u(32, uint32_t(1) << 30);
    // Cell mask: 2 sats × 1 sig = 2 bits, both present
    bw.write_u(1, 1);
    bw.write_u(1, 1);
    // Satellite data: DF397 × 2, then DF398 × 2
    bw.write_u(8, 70); bw.write_u(8, 80);    // rough int ms
    bw.write_u(10, 0); bw.write_u(10, 512);  // rough frac ms
    // Signal data: 2 cells
    bw.write_s(15, 0); bw.write_s(15, 0);
    bw.write_s(22, 0); bw.write_s(22, 0);
    bw.write_u(4, 3); bw.write_u(4, 7);
    bw.write_u(1, 0); bw.write_u(1, 0);
    bw.write_u(6, 35); bw.write_u(6, 42);

    auto msg = gnsspp::decode_msm4(bw.data());
    ASSERT_EQ(msg.satellites.size(), 2u);
    EXPECT_EQ(msg.satellites[0].id, 1);
    EXPECT_EQ(msg.satellites[1].id, 3);
    EXPECT_NEAR(msg.satellites[0].rough_range_ms, 70.0, 1e-6);
    EXPECT_NEAR(msg.satellites[1].rough_range_ms, 80.5, 1e-6);

    ASSERT_EQ(msg.signals.size(), 2u);
    EXPECT_EQ(msg.signals[0].sat_id, 1);
    EXPECT_EQ(msg.signals[1].sat_id, 3);
    EXPECT_EQ(msg.signals[0].lock_time_ind, 3);
    EXPECT_EQ(msg.signals[1].lock_time_ind, 7);
    EXPECT_FLOAT_EQ(msg.signals[0].cnr_dbhz, 35.0f);
    EXPECT_FLOAT_EQ(msg.signals[1].cnr_dbhz, 42.0f);
}

TEST(Msm4Test, TooShort)
{
    std::vector<uint8_t> short_payload(10, 0x00);
    EXPECT_THROW(gnsspp::decode_msm4(short_payload), std::runtime_error);
}
