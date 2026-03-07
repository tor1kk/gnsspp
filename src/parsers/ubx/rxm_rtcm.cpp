#include <cstring>
#include <stdexcept>

#include "ubx_util.hpp"
#include "gnsspp/error.hpp"
#include "gnsspp/parsers/ubx/rxm_rtcm.hpp"


namespace gnsspp {
namespace {

/// UBX-RXM-RTCM payload (8 bytes, version 2).
struct __attribute__((packed)) RxmRtcmRaw {
    uint8_t  version;     ///< Message version (2)
    uint8_t  flags;       ///< bit 0: crcFailed
    uint16_t reserved0;
    uint16_t ref_station; ///< Reference station ID from RTCM message
    uint16_t msg_type;    ///< RTCM message type (e.g. 1005, 1077)
};

static_assert(sizeof(RxmRtcmRaw) == 8, "RxmRtcmRaw size mismatch");

} // anonymous namespace


RxmRtcm decode_rxm_rtcm(const std::vector<uint8_t>& payload)
{
    if (payload.size() < sizeof(RxmRtcmRaw)) {
        throw ParseError("RXM-RTCM: payload too short");
    }

    RxmRtcmRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(raw));

    RxmRtcm out{};
    out.msg_type    = raw.msg_type;
    out.ref_station = raw.ref_station;
    out.crc_failed  = (raw.flags & 0x01) != 0;

    return out;
}


} // namespace gnsspp
