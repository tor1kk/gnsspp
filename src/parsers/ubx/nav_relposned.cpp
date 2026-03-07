#include <cstring>
#include <stdexcept>

#include "ubx_util.hpp"
#include "gnsspp/error.hpp"
#include "gnsspp/parsers/ubx/nav_relposned.hpp"


namespace gnsspp {
namespace {

/// UBX-NAV-RELPOSNED payload (64 bytes, version 1).
struct __attribute__((packed)) NavRelPosNedRaw {
    uint8_t  version;             ///< Message version (1)
    uint8_t  reserved0;
    uint16_t ref_station_id;      ///< Differential reference station ID
    uint32_t itow;                ///< GPS time of week [ms]
    int32_t  rel_pos_n;           ///< Relative position North [cm]
    int32_t  rel_pos_e;           ///< Relative position East  [cm]
    int32_t  rel_pos_d;           ///< Relative position Down  [cm]
    int32_t  rel_pos_length;      ///< Distance to base [cm]
    int32_t  rel_pos_heading;     ///< Heading to base [1e-5 deg]
    uint8_t  reserved1[4];
    int8_t   rel_pos_hp_n;        ///< High-precision part N [0.1 mm]
    int8_t   rel_pos_hp_e;        ///< High-precision part E [0.1 mm]
    int8_t   rel_pos_hp_d;        ///< High-precision part D [0.1 mm]
    int8_t   rel_pos_hp_length;   ///< High-precision part of length [0.1 mm]
    uint32_t acc_n;               ///< Accuracy N [0.1 mm]
    uint32_t acc_e;               ///< Accuracy E [0.1 mm]
    uint32_t acc_d;               ///< Accuracy D [0.1 mm]
    uint32_t acc_length;          ///< Accuracy of distance [0.1 mm]
    uint32_t acc_heading;         ///< Accuracy of heading [1e-5 deg]
    uint8_t  reserved2[4];
    uint32_t flags;               ///< Packed status flags
};

static_assert(sizeof(NavRelPosNedRaw) == 64, "NavRelPosNedRaw size mismatch");

} // anonymous namespace


NavRelPosNed decode_nav_relposned(const std::vector<uint8_t>& payload)
{
    if (payload.size() < sizeof(NavRelPosNedRaw)) {
        throw ParseError("NAV-RELPOSNED: payload too short");
    }

    NavRelPosNedRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(raw));

    NavRelPosNed out{};
    out.itow            = raw.itow;
    out.ref_station_id  = raw.ref_station_id;

    // Combined high-precision position: cm + 0.1mm HP → metres
    out.rel_pos_n_m   = raw.rel_pos_n * 0.01 + raw.rel_pos_hp_n * 0.0001;
    out.rel_pos_e_m   = raw.rel_pos_e * 0.01 + raw.rel_pos_hp_e * 0.0001;
    out.rel_pos_d_m   = raw.rel_pos_d * 0.01 + raw.rel_pos_hp_d * 0.0001;
    out.rel_pos_length_m  = raw.rel_pos_length  * 0.01 + raw.rel_pos_hp_length * 0.0001;
    out.rel_pos_heading_deg = raw.rel_pos_heading * 1e-5;

    out.acc_n_m       = raw.acc_n       * 0.0001f;
    out.acc_e_m       = raw.acc_e       * 0.0001f;
    out.acc_d_m       = raw.acc_d       * 0.0001f;
    out.acc_length_m  = raw.acc_length  * 0.0001f;
    out.acc_heading_deg = raw.acc_heading * 1e-5f;

    // Flags
    out.gnss_fix_ok            = (raw.flags >> 0) & 0x01;
    out.diff_soln              = (raw.flags >> 1) & 0x01;
    out.rel_pos_valid          = (raw.flags >> 2) & 0x01;
    out.carr_soln              = (raw.flags >> 3) & 0x03;
    out.ref_pos_miss           = (raw.flags >> 6) & 0x01;
    out.ref_obs_miss           = (raw.flags >> 7) & 0x01;
    out.rel_pos_heading_valid  = (raw.flags >> 8) & 0x01;

    return out;
}


} // namespace gnsspp
