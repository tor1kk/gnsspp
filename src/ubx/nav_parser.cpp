#include "gnsspp/error.hpp"
#include "nav_parser.hpp"

namespace gnsspp {

void nav_parse(uint8_t msg_id, Frame& frame)
{
    switch (msg_id) {
        case 0x03: frame.type = "NAV-STATUS";     break;
        case 0x07: frame.type = "NAV-PVT";       break;
        case 0x14: frame.type = "NAV-HPPOSLLH";  break;
        case 0x35: frame.type = "NAV-SAT";       break;
        case 0x3B: frame.type = "NAV-SVIN";      break;
        case 0x3C: frame.type = "NAV-RELPOSNED"; break;
        default:
            break;  // unknown class/id — leave frame.type empty 
    }
}

} // namespace gnsspp
