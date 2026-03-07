#include "gnsspp/ubx/ack.hpp"
#include "gnsspp/error.hpp"


namespace gnsspp {

AckAck decode_ack_ack(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 2)
        throw ParseError("ACK-ACK: payload too short");
    return {payload[0], payload[1]};
}

AckNak decode_ack_nak(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 2)
        throw ParseError("ACK-NAK: payload too short");
    return {payload[0], payload[1]};
}

} // namespace gnsspp
