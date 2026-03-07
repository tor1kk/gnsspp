#pragma once

#include <cstdint>
#include <vector>


namespace gnsspp {

/// Decoded RTCM 1230 — GLONASS L1 and L2 Code-Phase Biases.
struct Msg1230 {
    uint16_t station_id;     ///< Reference station ID

    /// Whether the code-phase biases below are valid and applicable.
    bool bias_indicator;

    /// One optional bias field.
    struct Bias {
        bool   valid;        ///< True if this bias was transmitted
        double value_m;      ///< Bias value [m]
    };

    Bias l1ca;  ///< L1 C/A code-phase bias
    Bias l1p;   ///< L1 P  code-phase bias
    Bias l2ca;  ///< L2 C/A code-phase bias
    Bias l2p;   ///< L2 P  code-phase bias
};

/// Decode an RTCM 1230 payload.
/// @param payload  Full RTCM3 payload bytes (message number bits included).
/// @throws gnsspp::ParseError if payload is too short or malformed.
Msg1230 decode_msg1230(const std::vector<uint8_t>& payload);

} // namespace gnsspp
