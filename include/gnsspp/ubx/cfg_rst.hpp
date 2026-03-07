#pragma once

#include <cstdint>
#include <vector>


namespace gnsspp {

/// navBbrMask values for UBX-CFG-RST.
enum CfgRstMask : uint16_t {
    CFG_RST_HOT   = 0x0000,  ///< Hot start — keep all nav data
    CFG_RST_WARM  = 0x0001,  ///< Warm start — clear ephemeris only
    CFG_RST_COLD  = 0xFFFF,  ///< Cold start — clear all nav data
};

/// resetMode values for UBX-CFG-RST.
enum CfgRstMode : uint8_t {
    CFG_RST_MODE_HW      = 0x00,  ///< Hardware reset (watchdog)
    CFG_RST_MODE_SW      = 0x01,  ///< Software reset (controlled)
    CFG_RST_MODE_GNSS    = 0x02,  ///< GNSS subsystem reset only
};

/// Build a UBX-CFG-RST frame ready to pass to Port::write().
/// @param nav_bbr_mask  see CfgRstMask
/// @param reset_mode    see CfgRstMode
std::vector<uint8_t> build_cfg_rst(uint16_t nav_bbr_mask, uint8_t reset_mode);

} // namespace gnsspp
