#include "gnsspp/ubx/ubx_poll.hpp"
#include "ubx_frame.hpp"


namespace gnsspp {

std::vector<uint8_t> build_ubx_poll(uint8_t cls, uint8_t msg_id)
{
    return detail::build_ubx_frame(cls, msg_id, {});
}

} // namespace gnsspp
