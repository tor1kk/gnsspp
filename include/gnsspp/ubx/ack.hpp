#pragma once

#include <cstdint>
#include <vector>


namespace gnsspp {

/// Decoded UBX-ACK-ACK: the module accepted the referenced message.
struct AckAck {
    uint8_t cls_id;  ///< Class ID of the acknowledged message
    uint8_t msg_id;  ///< Message ID of the acknowledged message
};

/// Decoded UBX-ACK-NAK: the module rejected the referenced message.
struct AckNak {
    uint8_t cls_id;  ///< Class ID of the rejected message
    uint8_t msg_id;  ///< Message ID of the rejected message
};

/// Decode a UBX-ACK-ACK payload (2 bytes).
/// @throws gnsspp::ParseError if payload is too short.
AckAck decode_ack_ack(const std::vector<uint8_t>& payload);

/// Decode a UBX-ACK-NAK payload (2 bytes).
/// @throws gnsspp::ParseError if payload is too short.
AckNak decode_ack_nak(const std::vector<uint8_t>& payload);

} // namespace gnsspp
