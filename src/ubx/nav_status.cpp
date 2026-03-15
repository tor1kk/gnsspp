#include <cstring>

#include "ubx_util.hpp"
#include "gnsspp/error.hpp"
#include "gnsspp/ubx/nav_status.hpp"

namespace gnsspp {
namespace {

struct __attribute__((packed)) NavStatusRaw {
    uint32_t itow;
    uint8_t  gps_fix;
    uint8_t  flags;     // bit0=gpsFixOk, bit1=diffSoln, bit2=wknSet, bit3=towSet
    uint8_t  fix_stat;  // bit0=diffCorr, bit2-3=carrSolnValid/carrSoln
    uint8_t  flags2;    // bit6-7=psmState, bit3-4=spoofDetState
    uint32_t ttff;      // ms
    uint32_t msss;      // ms
};

static_assert(sizeof(NavStatusRaw) == 16, "NavStatusRaw size mismatch");

} // anonymous namespace


NavStatus decode_nav_status(const std::vector<uint8_t>& payload)
{
    if (payload.size() < sizeof(NavStatusRaw)) {
        throw ParseError("NAV-STATUS: payload too short");
    }

    NavStatusRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(raw));

    NavStatus out{};
    out.itow             = raw.itow;
    out.gps_fix          = raw.gps_fix;
    out.gps_fix_ok       = (raw.flags    & 0x01) != 0;
    out.diff_soln        = (raw.flags    & 0x02) != 0;
    out.carr_soln_valid  = (raw.fix_stat & 0x04) != 0;
    out.carr_soln        = (raw.fix_stat >> 3) & 0x03;
    out.ttff             = raw.ttff;
    out.msss             = raw.msss;
    return out;
}


} // namespace gnsspp
