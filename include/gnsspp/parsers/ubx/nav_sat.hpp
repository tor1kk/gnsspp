#pragma once

#include <cstdint>
#include <vector>

namespace gnsspp {

/// Decoded information for a single tracked satellite (one entry in UBX-NAV-SAT).
struct SvInfo {
    uint8_t     gnss_id;        ///< GNSS constellation ID (0=GPS, 1=SBAS, 2=GAL, 3=BDS, 6=GLO)
    uint8_t     sv_id;          ///< Satellite (SV) ID within the constellation
    uint8_t     cno;            ///< Carrier-to-noise density [dBHz]
    int8_t      elev;           ///< Elevation angle [deg], range -90..90
    int16_t     azim;           ///< Azimuth angle [deg], range 0..360
    float       pr_res;         ///< Pseudorange residual [m] (raw I2 × 0.1)

    uint8_t     quality_ind;    ///< Signal quality indicator (bits 0-2 of flags)
    bool        sv_used;        ///< True if SV is used in the navigation solution
    uint8_t     health;         ///< SV health flag (0=healthy, 1=unhealthy, ...)
    bool        diff_corr;      ///< Differential correction available
    bool        smoothed;       ///< Carrier-smoothed pseudorange used

    uint8_t     orbit_source;   ///< Orbit source (0=no, 1=eph, 2=alm, 3=AssistNow, ...)
    bool        eph_avail;      ///< Ephemeris available
    bool        alm_avail;      ///< Almanac available
    bool        ano_avail;      ///< AssistNow Offline data available
    bool        aop_avail;      ///< AssistNow Autonomous data available

    bool        sbas_corr_used;    ///< SBAS corrections used
    bool        rtcm_corr_used;    ///< RTCM corrections used
    bool        slas_corr_used;    ///< QZSS SLAS corrections used
    bool        spartn_corr_used;  ///< SPARTN corrections used
    bool        pr_corr_used;      ///< Pseudorange correction used
    bool        cr_corr_used;      ///< Carrier range correction used
    bool        do_corr_used;      ///< Range rate (Doppler) correction used
    bool        clas_corr_used;    ///< CLAS corrections used
};

/// Decoded UBX-NAV-SAT message: satellite information for all tracked SVs.
struct NavSat {
    uint32_t            itow;    ///< GPS time of week [ms]
    uint8_t             num_svs; ///< Number of satellites in the svs vector
    std::vector<SvInfo> svs;     ///< Per-satellite information
};

/// Decode a raw UBX-NAV-SAT payload into a NavSat struct.
/// @throws gnsspp::ParseError if payload is too short or truncated.
NavSat decode_nav_sat(const std::vector<uint8_t>& payload);

} // namespace gnsspp