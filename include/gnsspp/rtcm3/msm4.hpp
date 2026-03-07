#pragma once

#include <cstdint>
#include <limits>
#include <vector>


namespace gnsspp {

/// One satellite entry in an MSM4 message.
struct Msm4Sat {
    uint8_t id;              ///< Satellite ID (1-indexed PRN / slot / SVN)
    double  rough_range_ms;  ///< Rough pseudorange [ms]; NaN if satellite slot invalid
};

/// One signal observation entry in an MSM4 message.
struct Msm4Signal {
    uint8_t sat_id;          ///< Satellite ID (1-indexed)
    uint8_t sig_id;          ///< Signal ID (1-indexed, GNSS-specific)
    double  pseudorange_m;   ///< Pseudorange [m]; NaN if fine correction invalid
    double  phase_range_m;   ///< Carrier-phase range [m]; NaN if fine correction invalid
    uint8_t lock_time_ind;   ///< Lock-time indicator (0-15)
    bool    half_cycle;      ///< Half-cycle ambiguity flag
    float   cnr_dbhz;        ///< Carrier-to-noise ratio [dBHz]
};

/// Decoded RTCM MSM4 message (types 1074 / 1084 / 1094 / 1124).
struct Msm4 {
    uint16_t msg_type;       ///< 1074=GPS, 1084=GLONASS, 1094=Galileo, 1124=BeiDou
    uint16_t station_id;
    uint32_t epoch_time_ms;  ///< GNSS-specific time of week (or day for GLONASS) [ms]
    bool     multiple_msg;   ///< More MSM messages follow for this epoch

    std::vector<Msm4Sat>    satellites;
    std::vector<Msm4Signal> signals;
};

/// Decode an RTCM MSM4 payload (message types 1074, 1084, 1094, 1124).
/// @param payload  Full RTCM3 payload bytes (message number bits included).
/// @throws gnsspp::ParseError if payload is malformed or too short.
Msm4 decode_msm4(const std::vector<uint8_t>& payload);

} // namespace gnsspp
