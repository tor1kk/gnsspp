#pragma once

#include <cstdint>

#include "gnsspp/frame.hpp"

namespace gnsspp {

// Internal — dispatch NAV-class message (class=0x01) by msg_id.
// Unknown msg_id is silently ignored (frame.type left empty).
void nav_parse(uint8_t msg_id, Frame& frame);

} // namespace gnsspp
