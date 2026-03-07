#include <cstring>
#include <stdexcept>

#include "ubx_util.hpp"
#include "gnsspp/error.hpp"
#include "gnsspp/parsers/ubx/nav_pvt.hpp"

namespace gnsspp {
namespace {

/// NAV-PVT raw payload layout — mirrors the wire format exactly (little-endian).
/// Implementation detail: kept here to avoid polluting the public header.
struct __attribute__((packed)) NavPvtRaw {
    uint32_t itow;
    uint16_t year;
    uint8_t  month, day, hour, min, second;
    uint8_t  valid;         // X1 bitmask
    uint32_t t_acc;
    int32_t  nano;
    uint8_t  fix_type;
    uint8_t  flags;         // X1 bitmask: bit0=gnssFixOk, bits6-7=carrSoln
    uint8_t  flags2;
    uint8_t  num_sv;
    int32_t  lon_raw;       // degrees * 1e7
    int32_t  lat_raw;       // degrees * 1e7
    int32_t  height;        // mm
    int32_t  h_msl;         // mm
    uint32_t h_acc;         // mm
    uint32_t v_acc;         // mm
    int32_t  vel_n;         // mm/s
    int32_t  vel_e;         // mm/s
    int32_t  vel_d;         // mm/s
    int32_t  g_speed;       // mm/s
    int32_t  head_mot_raw;  // degrees * 1e5
    uint32_t s_acc;         // mm/s
    uint32_t head_acc_raw;  // degrees * 1e5
    uint16_t p_dop_raw;     // * 0.01
    // [78..91] flags3, reserved0, headVeh, magDec, magAcc — not decoded
};

static_assert(sizeof(NavPvtRaw) == 78, "NavPvtRaw size mismatch");

} // anonymous namespace


NavPvt decode_nav_pvt(const std::vector<uint8_t>& payload)
{
    if (payload.size() < sizeof(NavPvtRaw)) {
        throw ParseError("NAV-PVT: payload too short");
    }

    NavPvtRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(raw));

    NavPvt out{};

    out.itow   = raw.itow;
    out.year   = raw.year;
    out.month  = raw.month;
    out.day    = raw.day;
    out.hour   = raw.hour;
    out.min    = raw.min;
    out.second = raw.second;

    out.fix_type = raw.fix_type;
    out.num_sv   = raw.num_sv;

    out.gnss_fix_ok = (raw.flags & 0x01) != 0;
    out.carr_soln   = (raw.flags >> 6) & 0x03;

    out.lon = raw.lon_raw * 1e-7;
    out.lat = raw.lat_raw * 1e-7;

    out.height  = raw.height;
    out.h_msl   = raw.h_msl;
    out.h_acc   = raw.h_acc;
    out.v_acc   = raw.v_acc;

    out.vel_n   = raw.vel_n;
    out.vel_e   = raw.vel_e;
    out.vel_d   = raw.vel_d;
    out.g_speed = raw.g_speed;
    out.s_acc   = raw.s_acc;

    out.head_mot = raw.head_mot_raw * 1e-5;
    out.p_dop    = raw.p_dop_raw    * 0.01;

    return out;
}


} // namespace gnsspp
