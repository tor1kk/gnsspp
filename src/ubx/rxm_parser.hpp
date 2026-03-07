#pragma once

#include <cstdint>

#include "gnsspp/frame.hpp"


namespace gnsspp {

/// Dispatch a UBX-RXM message to the appropriate decoder.
/// @param msg_id  Second byte of the UBX message ID (class byte is 0x02).
/// @param payload Raw payload bytes.
/// @param frame   Output frame; frame.parsed is populated on success.
/// Unknown msg_id is silently ignored (frame.type left empty).
void rxm_parse(uint8_t msg_id, Frame& frame);

} // namespace gnsspp
