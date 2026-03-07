#include <gtest/gtest.h>
#include "gnsspp/ubx/cfg_valset.hpp"


namespace {

// Compute UBX Fletcher checksum over bytes [2..N-3] (cls through payload)
void calc_checksum(const std::vector<uint8_t>& frame, uint8_t& ck_a, uint8_t& ck_b)
{
    ck_a = 0; ck_b = 0;
    // checksum covers bytes 2..size-3 (skip sync 0xB5,0x62 and two checksum bytes)
    for (size_t i = 2; i < frame.size() - 2; ++i) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
}

TEST(CfgValset, SyncAndClassId)
{
    auto frame = gnsspp::build_cfg_valset({}, gnsspp::CFG_LAYER_RAM);

    ASSERT_GE(frame.size(), 8u);
    EXPECT_EQ(frame[0], 0xB5);  // sync1
    EXPECT_EQ(frame[1], 0x62);  // sync2
    EXPECT_EQ(frame[2], 0x06);  // class CFG
    EXPECT_EQ(frame[3], 0x8A);  // id CFG-VALSET
}

TEST(CfgValset, EmptyItemsLength)
{
    // Empty items: payload = version(1) + layers(1) + reserved(2) = 4 bytes
    auto frame = gnsspp::build_cfg_valset({}, gnsspp::CFG_LAYER_RAM);
    uint16_t len = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
    EXPECT_EQ(len, 4u);
    // Total frame = 2(sync) + 2(cls+id) + 2(len) + 4(payload) + 2(ck) = 12
    EXPECT_EQ(frame.size(), 12u);
}

TEST(CfgValset, LayerInPayload)
{
    auto frame_ram   = gnsspp::build_cfg_valset({}, gnsspp::CFG_LAYER_RAM);
    auto frame_flash = gnsspp::build_cfg_valset({}, gnsspp::CFG_LAYER_FLASH);

    EXPECT_EQ(frame_ram[7],   gnsspp::CFG_LAYER_RAM);    // payload byte 1 = layers
    EXPECT_EQ(frame_flash[7], gnsspp::CFG_LAYER_FLASH);
}

TEST(CfgValset, U1ValueEncoding)
{
    // CFG_MSGOUT_UBX_NAV_PVT_USB = 0x20910007 — size type 0x2 → 1 byte
    const uint32_t key = 0x20910007;
    gnsspp::CfgVal item{key, 1};
    auto frame = gnsspp::build_cfg_valset({item}, gnsspp::CFG_LAYER_RAM);

    // Payload: version(1)+layers(1)+reserved(2)+key(4)+value(1) = 9
    uint16_t len = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
    EXPECT_EQ(len, 9u);

    // Key at payload offset 4 (after sync+cls+id+len+4 header bytes)
    const size_t key_off = 6 + 4;
    EXPECT_EQ(frame[key_off + 0],  key        & 0xFF);
    EXPECT_EQ(frame[key_off + 1], (key >>  8) & 0xFF);
    EXPECT_EQ(frame[key_off + 2], (key >> 16) & 0xFF);
    EXPECT_EQ(frame[key_off + 3], (key >> 24) & 0xFF);
    EXPECT_EQ(frame[key_off + 4], 0x01);  // value = 1
}

TEST(CfgValset, U4ValueEncoding)
{
    // CFG_TMODE_SVIN_MIN_DUR = 0x40030010 — size type 0x4 → 4 bytes
    const uint32_t key = 0x40030010;
    gnsspp::CfgVal item{key, 0x0000003C};  // 60 seconds
    auto frame = gnsspp::build_cfg_valset({item}, gnsspp::CFG_LAYER_RAM);

    uint16_t len = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
    EXPECT_EQ(len, 12u);  // 4 header + 4 key + 4 value

    const size_t val_off = 6 + 4 + 4;
    EXPECT_EQ(frame[val_off + 0], 0x3C);
    EXPECT_EQ(frame[val_off + 1], 0x00);
    EXPECT_EQ(frame[val_off + 2], 0x00);
    EXPECT_EQ(frame[val_off + 3], 0x00);
}

TEST(CfgValset, ChecksumValid)
{
    gnsspp::CfgVal item{0x20910007, 1};
    auto frame = gnsspp::build_cfg_valset({item}, gnsspp::CFG_LAYER_RAM);

    uint8_t ck_a, ck_b;
    calc_checksum(frame, ck_a, ck_b);

    EXPECT_EQ(frame[frame.size() - 2], ck_a);
    EXPECT_EQ(frame[frame.size() - 1], ck_b);
}

TEST(CfgValset, MultipleItems)
{
    std::vector<gnsspp::CfgVal> items = {
        {0x20910007, 1},  // U1
        {0x40030010, 60}, // U4
    };
    auto frame = gnsspp::build_cfg_valset(items, gnsspp::CFG_LAYER_RAM);

    // payload = 4 + (4+1) + (4+4) = 17
    uint16_t len = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
    EXPECT_EQ(len, 17u);

    uint8_t ck_a, ck_b;
    calc_checksum(frame, ck_a, ck_b);
    EXPECT_EQ(frame[frame.size() - 2], ck_a);
    EXPECT_EQ(frame[frame.size() - 1], ck_b);
}

} // namespace
