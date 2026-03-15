#include <gtest/gtest.h>
#include "gnsspp/ubx/cfg_valget.hpp"


namespace {

void calc_checksum(const std::vector<uint8_t>& frame, uint8_t& ck_a, uint8_t& ck_b)
{
    ck_a = 0; ck_b = 0;
    for (size_t i = 2; i < frame.size() - 2; ++i) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
}

// ---------------------------------------------------------------------------
// build_cfg_valget
// ---------------------------------------------------------------------------

TEST(CfgValget, SyncAndClassId)
{
    auto frame = gnsspp::build_cfg_valget({}, gnsspp::CFG_GET_LAYER_RAM);

    ASSERT_GE(frame.size(), 8u);
    EXPECT_EQ(frame[0], 0xB5);  // sync1
    EXPECT_EQ(frame[1], 0x62);  // sync2
    EXPECT_EQ(frame[2], 0x06);  // class CFG
    EXPECT_EQ(frame[3], 0x8B);  // id CFG-VALGET
}

TEST(CfgValget, EmptyKeysLength)
{
    // No keys: payload = version(1) + layer(1) + position(2) = 4 bytes
    auto frame = gnsspp::build_cfg_valget({});
    uint16_t len = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
    EXPECT_EQ(len, 4u);
    EXPECT_EQ(frame.size(), 12u);  // 2+2+2+4+2
}

TEST(CfgValget, LayerInPayload)
{
    auto frame_ram   = gnsspp::build_cfg_valget({}, gnsspp::CFG_GET_LAYER_RAM);
    auto frame_flash = gnsspp::build_cfg_valget({}, gnsspp::CFG_GET_LAYER_FLASH);
    auto frame_def   = gnsspp::build_cfg_valget({}, gnsspp::CFG_GET_LAYER_DEFAULT);

    EXPECT_EQ(frame_ram[7],   static_cast<uint8_t>(gnsspp::CFG_GET_LAYER_RAM));
    EXPECT_EQ(frame_flash[7], static_cast<uint8_t>(gnsspp::CFG_GET_LAYER_FLASH));
    EXPECT_EQ(frame_def[7],   static_cast<uint8_t>(gnsspp::CFG_GET_LAYER_DEFAULT));
}

TEST(CfgValget, PositionInPayload)
{
    auto frame = gnsspp::build_cfg_valget({}, gnsspp::CFG_GET_LAYER_RAM, 0x0102);

    EXPECT_EQ(frame[8], 0x02);  // position lo
    EXPECT_EQ(frame[9], 0x01);  // position hi
}

TEST(CfgValget, SingleKeyEncoding)
{
    const uint32_t key = 0x20910009;
    auto frame = gnsspp::build_cfg_valget({key}, gnsspp::CFG_GET_LAYER_RAM);

    // payload = 4 header + 4 key = 8
    uint16_t len = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
    EXPECT_EQ(len, 8u);

    const size_t key_off = 6 + 4;
    EXPECT_EQ(frame[key_off + 0],  key        & 0xFF);
    EXPECT_EQ(frame[key_off + 1], (key >>  8) & 0xFF);
    EXPECT_EQ(frame[key_off + 2], (key >> 16) & 0xFF);
    EXPECT_EQ(frame[key_off + 3], (key >> 24) & 0xFF);
}

TEST(CfgValget, MultipleKeysLength)
{
    auto frame = gnsspp::build_cfg_valget({0x20910009, 0x40030010, 0x20030001});
    uint16_t len = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
    EXPECT_EQ(len, 4u + 3 * 4u);  // 4 header + 3 keys × 4 bytes
}

TEST(CfgValget, ChecksumValid)
{
    auto frame = gnsspp::build_cfg_valget({0x20910009, 0x40030010});

    uint8_t ck_a, ck_b;
    calc_checksum(frame, ck_a, ck_b);

    EXPECT_EQ(frame[frame.size() - 2], ck_a);
    EXPECT_EQ(frame[frame.size() - 1], ck_b);
}

// ---------------------------------------------------------------------------
// parse_cfg_valget
// ---------------------------------------------------------------------------

TEST(CfgValget, ParseEmptyPayload)
{
    auto result = gnsspp::parse_cfg_valget({});
    EXPECT_TRUE(result.empty());
}

TEST(CfgValget, ParseTooShortPayload)
{
    // Less than 4-byte header
    auto result = gnsspp::parse_cfg_valget({0x00, 0x00});
    EXPECT_TRUE(result.empty());
}

TEST(CfgValget, ParseU1Value)
{
    // version(1) + layer(1) + position(2) + key(4) + value(1)
    // key 0x20910009 — size type 0x2 → 1 byte
    const uint32_t key = 0x20910009;
    std::vector<uint8_t> payload = {
        0x00,                               // version
        0x00,                               // layer RAM
        0x00, 0x00,                         // position
        static_cast<uint8_t>( key        & 0xFF),
        static_cast<uint8_t>((key >>  8) & 0xFF),
        static_cast<uint8_t>((key >> 16) & 0xFF),
        static_cast<uint8_t>((key >> 24) & 0xFF),
        0x01,                               // value = 1
    };

    auto result = gnsspp::parse_cfg_valget(payload);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].key,   key);
    EXPECT_EQ(result[0].value, 1u);
}

TEST(CfgValget, ParseU4Value)
{
    // key 0x40030010 — size type 0x4 → 4 bytes; value = 60 (0x3C)
    const uint32_t key = 0x40030010;
    std::vector<uint8_t> payload = {
        0x00, 0x00, 0x00, 0x00,             // header
        static_cast<uint8_t>( key        & 0xFF),
        static_cast<uint8_t>((key >>  8) & 0xFF),
        static_cast<uint8_t>((key >> 16) & 0xFF),
        static_cast<uint8_t>((key >> 24) & 0xFF),
        0x3C, 0x00, 0x00, 0x00,             // value = 60 LE
    };

    auto result = gnsspp::parse_cfg_valget(payload);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].key,   key);
    EXPECT_EQ(result[0].value, 60u);
}

TEST(CfgValget, ParseMultipleValues)
{
    // Two items: U1 key + U4 key
    const uint32_t k1 = 0x20910009;  // U1
    const uint32_t k2 = 0x40030010;  // U4
    std::vector<uint8_t> payload = {
        0x00, 0x00, 0x00, 0x00,
        // k1 + value 1
        static_cast<uint8_t>( k1        & 0xFF),
        static_cast<uint8_t>((k1 >>  8) & 0xFF),
        static_cast<uint8_t>((k1 >> 16) & 0xFF),
        static_cast<uint8_t>((k1 >> 24) & 0xFF),
        0x01,
        // k2 + value 120
        static_cast<uint8_t>( k2        & 0xFF),
        static_cast<uint8_t>((k2 >>  8) & 0xFF),
        static_cast<uint8_t>((k2 >> 16) & 0xFF),
        static_cast<uint8_t>((k2 >> 24) & 0xFF),
        0x78, 0x00, 0x00, 0x00,
    };

    auto result = gnsspp::parse_cfg_valget(payload);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0].key,   k1);
    EXPECT_EQ(result[0].value, 1u);
    EXPECT_EQ(result[1].key,   k2);
    EXPECT_EQ(result[1].value, 120u);
}

} // namespace
