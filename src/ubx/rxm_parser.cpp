#include "gnsspp/error.hpp"
#include "rxm_parser.hpp"

namespace gnsspp {

void rxm_parse(uint8_t msg_id, Frame& frame)
{
    switch (msg_id) {
        case 0x32: frame.type = "RXM-RTCM"; break;
        default:
            break;  // unknown class/id — leave frame.type empty
    }
}

} // namespace gnsspp
