#include <cstring>

#include "ubx_util.hpp"
#include "gnsspp/error.hpp"
#include "gnsspp/ubx/nav_hpposllh.hpp"

namespace gnsspp {
namespace {

struct __attribute__((packed)) NavHpPosLlhRaw {
    uint8_t  version;       // always 0
    uint8_t  reserved1[2];
    uint8_t  flags;         // bit0 = invalidLlh
    uint32_t itow;          // ms
    int32_t  lon_raw;       // degrees * 1e7
    int32_t  lat_raw;       // degrees * 1e7
    int32_t  height_raw;    // mm
    int32_t  h_msl_raw;     // mm
    int8_t   lon_hp;        // degrees * 1e9 (signed, range -99..+99)
    int8_t   lat_hp;        // degrees * 1e9
    int8_t   height_hp;     // 0.1 mm
    int8_t   h_msl_hp;      // 0.1 mm
    uint32_t h_acc_raw;     // 0.1 mm
    uint32_t v_acc_raw;     // 0.1 mm
};

static_assert(sizeof(NavHpPosLlhRaw) == 36, "NavHpPosLlhRaw size mismatch");

} // anonymous namespace


NavHpPosLlh decode_nav_hpposllh(const std::vector<uint8_t>& payload)
{
    if (payload.size() < sizeof(NavHpPosLlhRaw)) {
        throw ParseError("NAV-HPPOSLLH: payload too short");
    }

    NavHpPosLlhRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(raw));

    NavHpPosLlh out{};
    out.itow        = raw.itow;
    out.invalid_llh = (raw.flags & 0x01) != 0;
    out.lon         = raw.lon_raw    * 1e-7 + raw.lon_hp    * 1e-9;
    out.lat         = raw.lat_raw    * 1e-7 + raw.lat_hp    * 1e-9;
    out.height      = raw.height_raw + raw.height_hp * 0.1;
    out.h_msl       = raw.h_msl_raw  + raw.h_msl_hp  * 0.1;
    out.h_acc       = raw.h_acc_raw  * 0.1;
    out.v_acc       = raw.v_acc_raw  * 0.1;
    return out;
}


} // namespace gnsspp
