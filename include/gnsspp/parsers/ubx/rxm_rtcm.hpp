#pragma once

#include <cstdint>
#include <vector>

namespace gnsspp {

/// Decoded UBX-RXM-RTCM message: status of an incoming RTCM correction message.
/// Emitted by the receiver each time an RTCM message is processed.
struct RxmRtcm {
    uint16_t msg_type;    ///< RTCM message type (e.g. 1005, 1077, ...)
    uint16_t ref_station; ///< Reference station ID from the RTCM message
    bool     crc_failed;  ///< True if the incoming RTCM message had a CRC error
};

/// Decode a raw UBX-RXM-RTCM payload into an RxmRtcm struct.
/// @throws gnsspp::ParseError if payload is too short.
RxmRtcm decode_rxm_rtcm(const std::vector<uint8_t>& payload);

} // namespace gnsspp
