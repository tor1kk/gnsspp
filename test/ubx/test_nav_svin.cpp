#include <gtest/gtest.h>
#include "gnsspp/ubx/nav_svin.hpp"


class NavSvInTest : public ::testing::Test {
protected:
    // NavSvInRaw (40 bytes, version 0):
    //   version(1) + reserved0(3) + itow(4) + dur(4)
    //   + mean_x(4) + mean_y(4) + mean_z(4)
    //   + mean_x_hp(1) + mean_y_hp(1) + mean_z_hp(1) + reserved1(1)
    //   + mean_acc(4) + obs(4) + valid(1) + active(1) + reserved2(2)
    std::vector<uint8_t> payload = {
        0x00,                   // version = 0
        0x00, 0x00, 0x00,       // reserved0
        0xE0, 0x93, 0x04, 0x00, // itow = 300000 ms
        0x78, 0x00, 0x00, 0x00, // dur = 120 s
        0xA0, 0x86, 0x01, 0x00, // mean_x = 100000 cm  → 1000.0 m
        0x40, 0x0D, 0x03, 0x00, // mean_y = 200000 cm  → 2000.0 m
        0x50, 0xC3, 0x00, 0x00, // mean_z =  50000 cm  →  500.0 m
        0x05,                   // mean_x_hp = +5 (0.1mm) → +0.0005 m
        0x03,                   // mean_y_hp = +3         → +0.0003 m
        0xFE,                   // mean_z_hp = -2         → -0.0002 m
        0x00,                   // reserved1
        0xDC, 0x05, 0x00, 0x00, // mean_acc = 1500 (0.1 mm) → 0.15 m
        0xFA, 0x00, 0x00, 0x00, // obs = 250
        0x01,                   // valid = 1
        0x00,                   // active = 0
        0x00, 0x00,             // reserved2
    };
};


TEST_F(NavSvInTest, Decode) {
    auto msg = gnsspp::decode_nav_svin(payload);

    EXPECT_EQ(msg.itow,  300000u);
    EXPECT_EQ(msg.dur,   120u);
    EXPECT_NEAR(msg.mean_x_m,   1000.0005, 1e-6);
    EXPECT_NEAR(msg.mean_y_m,   2000.0003, 1e-6);
    EXPECT_NEAR(msg.mean_z_m,    499.9998, 1e-6);
    EXPECT_NEAR(msg.mean_acc_m,  0.15f,    1e-4f);
    EXPECT_EQ(msg.obs,   250u);
    EXPECT_TRUE(msg.valid);
    EXPECT_FALSE(msg.active);
}

TEST_F(NavSvInTest, TooShort) {
    std::vector<uint8_t> short_payload = { 0x00, 0x01, 0x02 };
    EXPECT_THROW(gnsspp::decode_nav_svin(short_payload), std::runtime_error);
}
