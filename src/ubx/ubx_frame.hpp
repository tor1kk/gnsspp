#pragma once

#include <cstdint>
#include <vector>


namespace gnsspp {
namespace detail {

/// Fletcher-8 checksum over [cls, id, len_lo, len_hi, payload...].
inline void ubx_checksum(const uint8_t* data, size_t len,
                         uint8_t& ck_a, uint8_t& ck_b)
{
    ck_a = 0;
    ck_b = 0;
    for (size_t i = 0; i < len; ++i) {
        ck_a += data[i];
        ck_b += ck_a;
    }
}

/// Build a complete UBX frame: 0xB5 0x62 + cls + id + len + payload + CK_A + CK_B.
inline std::vector<uint8_t> build_ubx_frame(uint8_t cls, uint8_t id,
                                             const std::vector<uint8_t>& payload)
{
    const auto plen = static_cast<uint16_t>(payload.size());

    // Checksum is over: cls, id, len_lo, len_hi, payload
    std::vector<uint8_t> chk;
    chk.reserve(4 + plen);
    chk.push_back(cls);
    chk.push_back(id);
    chk.push_back(plen & 0xFF);
    chk.push_back((plen >> 8) & 0xFF);
    chk.insert(chk.end(), payload.begin(), payload.end());

    uint8_t ck_a, ck_b;
    ubx_checksum(chk.data(), chk.size(), ck_a, ck_b);

    std::vector<uint8_t> frame;
    frame.reserve(2 + chk.size() + 2);
    frame.push_back(0xB5);
    frame.push_back(0x62);
    frame.insert(frame.end(), chk.begin(), chk.end());
    frame.push_back(ck_a);
    frame.push_back(ck_b);

    return frame;
}

/// Extract value size in bytes from a CFG-VALSET key (bits 28-30).
inline size_t cfg_key_size(uint32_t key)
{
    const uint8_t size_type = (key >> 28) & 0x07;
    switch (size_type) {
        case 0x01: return 1;  // L (bool, 1 bit stored as 1 byte)
        case 0x02: return 1;  // U1/I1/X1/E1
        case 0x03: return 2;  // U2/I2/X2/E2
        case 0x04: return 4;  // U4/I4/X4/R4/E4
        case 0x05: return 8;  // U8/I8/X8/R8
        default:   return 0;  // unknown
    }
}

} // namespace detail
} // namespace gnsspp
