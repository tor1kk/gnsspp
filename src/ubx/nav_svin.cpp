#include <cstring>
#include <stdexcept>

#include "ubx_util.hpp"
#include "gnsspp/error.hpp"
#include "gnsspp/ubx/nav_svin.hpp"


namespace gnsspp {
namespace {

/// UBX-NAV-SVIN payload (40 bytes, version 0).
struct __attribute__((packed)) NavSvInRaw {
    uint8_t  version;       ///< Message version (0)
    uint8_t  reserved0[3];
    uint32_t itow;          ///< GPS time of week [ms]
    uint32_t dur;           ///< Elapsed survey-in time [s]
    int32_t  mean_x;        ///< Mean ECEF X [cm]
    int32_t  mean_y;        ///< Mean ECEF Y [cm]
    int32_t  mean_z;        ///< Mean ECEF Z [cm]
    int8_t   mean_x_hp;     ///< High-precision part of X [0.1 mm, −9..+9]
    int8_t   mean_y_hp;     ///< High-precision part of Y [0.1 mm]
    int8_t   mean_z_hp;     ///< High-precision part of Z [0.1 mm]
    uint8_t  reserved1;
    uint32_t mean_acc;      ///< Mean accuracy estimate [0.1 mm]
    uint32_t obs;           ///< Observations used
    uint8_t  valid;         ///< Survey-in position valid flag
    uint8_t  active;        ///< Survey-in in progress flag
    uint8_t  reserved2[2];
};

static_assert(sizeof(NavSvInRaw) == 40, "NavSvInRaw size mismatch");

} // anonymous namespace


NavSvIn decode_nav_svin(const std::vector<uint8_t>& payload)
{
    if (payload.size() < sizeof(NavSvInRaw)) {
        throw ParseError("NAV-SVIN: payload too short");
    }

    NavSvInRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(raw));

    NavSvIn out{};
    out.itow       = raw.itow;
    out.dur        = raw.dur;
    // Combined high-precision position: cm + 0.1mm HP → metres
    out.mean_x_m   = raw.mean_x * 0.01 + raw.mean_x_hp * 0.0001;
    out.mean_y_m   = raw.mean_y * 0.01 + raw.mean_y_hp * 0.0001;
    out.mean_z_m   = raw.mean_z * 0.01 + raw.mean_z_hp * 0.0001;
    out.mean_acc_m = raw.mean_acc * 0.0001f;  // 0.1mm → m
    out.obs        = raw.obs;
    out.valid      = raw.valid != 0;
    out.active     = raw.active != 0;

    return out;
}


} // namespace gnsspp
