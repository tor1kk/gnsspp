#include "gnsspp/ubx/cfg_rst.hpp"
#include "ubx_frame.hpp"


namespace gnsspp {

// CFG-RST: class 0x06, id 0x04
static constexpr uint8_t CLASS_CFG   = 0x06;
static constexpr uint8_t ID_CFG_RST  = 0x04;


std::vector<uint8_t> build_cfg_rst(uint16_t nav_bbr_mask, uint8_t reset_mode)
{
    std::vector<uint8_t> payload = {
        static_cast<uint8_t>( nav_bbr_mask        & 0xFF),  // navBbrMask lo
        static_cast<uint8_t>((nav_bbr_mask >> 8)  & 0xFF),  // navBbrMask hi
        reset_mode,
        0x00,  // reserved
    };

    return detail::build_ubx_frame(CLASS_CFG, ID_CFG_RST, payload);
}

} // namespace gnsspp
