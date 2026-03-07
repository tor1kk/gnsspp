#include <cstring>
#include <stdexcept>

#include "ubx_util.hpp"
#include "gnsspp/error.hpp"
#include "gnsspp/ubx/nav_sat.hpp"


namespace gnsspp {
namespace {

/// Fixed 8-byte header at the start of every UBX-NAV-SAT payload.
struct __attribute__((packed)) NavSatHeader {
    uint32_t itow;      ///< GPS time of week [ms]
    uint8_t  version;   ///< Message version (should be 1)
    uint8_t  num_svs;   ///< Number of SvInfoRaw records that follow
    uint16_t reserved;
};

/// One 12-byte record per tracked satellite, following NavSatHeader.
struct __attribute__((packed)) SvInfoRaw {
    uint8_t  gnss_id;
    uint8_t  sv_id;
    uint8_t  cno;
    int8_t   elev;
    int16_t  azim;
    int16_t  pr_res_raw; ///< Pseudorange residual × 10 (scale factor 0.1 m)
    uint32_t flags;      ///< Packed bitfield — see UBX-NAV-SAT interface description
};

static_assert(sizeof(NavSatHeader) == 8,  "NavSatHeader size mismatch");
static_assert(sizeof(SvInfoRaw)    == 12, "SvInfoRaw size mismatch");

} // anonymous namespace


/// Decode a raw UBX-NAV-SAT payload.
/// Copies the fixed header, then iterates over each SvInfoRaw record,
/// extracting and scaling all fields into the public SvInfo struct.
NavSat decode_nav_sat(const std::vector<uint8_t>& payload)
{
    if (payload.size() < sizeof(NavSatHeader)) {
        throw ParseError("NAV-SAT: payload too short");
    }

    NavSatHeader header{};
    std::memcpy(&header, payload.data(), sizeof(header));

    size_t expected = sizeof(NavSatHeader) + header.num_svs * sizeof(SvInfoRaw);
    if (payload.size() < expected) {
        throw ParseError("NAV-SAT: payload truncated");
    }

    NavSat out{};
    out.itow    = header.itow;
    out.num_svs = header.num_svs;
    out.svs.reserve(header.num_svs);

    for (size_t i = 0; i < out.num_svs; i++) {
        SvInfoRaw raw{};
        size_t offset = sizeof(NavSatHeader) + i * sizeof(SvInfoRaw);
        std::memcpy(&raw, payload.data() + offset, sizeof(SvInfoRaw));

        SvInfo svinfo{};
        svinfo.gnss_id       = raw.gnss_id;
        svinfo.sv_id         = raw.sv_id;
        svinfo.cno           = raw.cno;
        svinfo.elev          = raw.elev;
        svinfo.azim          = raw.azim;
        svinfo.pr_res        = raw.pr_res_raw * 0.1f;
        svinfo.quality_ind   = raw.flags & 0x07;
        svinfo.sv_used       = (raw.flags >> 3) & 0x01;    
        svinfo.health        = (raw.flags >> 4) & 0x03;    
        svinfo.diff_corr     = (raw.flags >> 6) & 0x01;    
        svinfo.smoothed      = (raw.flags >> 7) & 0x01;   
        svinfo.orbit_source  = (raw.flags >> 8) & 0x07;    
        svinfo.eph_avail     = (raw.flags >> 11) & 0x01;
        svinfo.alm_avail     = (raw.flags >> 12) & 0x01;
        svinfo.ano_avail     = (raw.flags >> 13) & 0x01;
        svinfo.aop_avail     = (raw.flags >> 14) & 0x01;
        // bit 15: reserved
        svinfo.sbas_corr_used    = (raw.flags >> 16) & 0x01;
        svinfo.rtcm_corr_used    = (raw.flags >> 17) & 0x01;
        svinfo.slas_corr_used    = (raw.flags >> 18) & 0x01;
        svinfo.spartn_corr_used  = (raw.flags >> 19) & 0x01;
        svinfo.pr_corr_used      = (raw.flags >> 20) & 0x01;
        svinfo.cr_corr_used      = (raw.flags >> 21) & 0x01;
        svinfo.do_corr_used      = (raw.flags >> 22) & 0x01;
        svinfo.clas_corr_used    = (raw.flags >> 23) & 0x01;

        out.svs.push_back(svinfo);
    }

    return out;
}


} // namespace gnsspp