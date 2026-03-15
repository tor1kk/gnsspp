#pragma once

#include <cstdint>
#include <vector>


namespace gnsspp {

/// Build a UBX poll request frame (empty payload).
/// Sending this to a u-blox receiver causes it to reply with the requested
/// message (class/id) populated with current values.
std::vector<uint8_t> build_ubx_poll(uint8_t cls, uint8_t msg_id);

} // namespace gnsspp
