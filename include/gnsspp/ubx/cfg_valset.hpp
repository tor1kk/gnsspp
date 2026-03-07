#pragma once

#include <cstdint>
#include <vector>


namespace gnsspp {

/// Layer bitmask for UBX-CFG-VALSET / CFG-VALGET.
enum CfgLayer : uint8_t {
    CFG_LAYER_RAM   = 0x01,  ///< Volatile RAM (lost on power cycle)
    CFG_LAYER_BBR   = 0x02,  ///< Battery-backed RAM
    CFG_LAYER_FLASH = 0x04,  ///< Non-volatile flash
    CFG_LAYER_ALL   = 0x07,  ///< All three layers
};

/// One key-value pair for UBX-CFG-VALSET.
/// The value size (1/2/4/8 bytes) is encoded in bits 28-30 of the key.
struct CfgVal {
    uint32_t key;    ///< u-blox configuration key ID (encodes size in bits 28-30)
    uint64_t value;  ///< Configuration value (right-aligned, only relevant bytes are written)
};

/// Build a UBX-CFG-VALSET frame ready to pass to Port::write().
/// @param items   list of key-value pairs to set
/// @param layers  bitmask of target layers (see CfgLayer)
std::vector<uint8_t> build_cfg_valset(const std::vector<CfgVal>& items,
                                      uint8_t layers = CFG_LAYER_RAM);

} // namespace gnsspp
