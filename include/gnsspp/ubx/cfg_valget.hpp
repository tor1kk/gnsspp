#pragma once

#include <cstdint>
#include <vector>

#include "gnsspp/ubx/cfg_valset.hpp"  // CfgVal


namespace gnsspp {

/// Layer selector for UBX-CFG-VALGET (single value, not a bitmask).
enum CfgGetLayer : uint8_t {
    CFG_GET_LAYER_RAM     = 0,  ///< Volatile RAM
    CFG_GET_LAYER_BBR     = 1,  ///< Battery-backed RAM
    CFG_GET_LAYER_FLASH   = 2,  ///< Non-volatile flash
    CFG_GET_LAYER_DEFAULT = 7,  ///< Effective (default) value
};

/// Build a UBX-CFG-VALGET request frame ready to pass to Port::write().
/// The receiver will reply with a UBX-CFG-VALGET response containing the
/// current values; parse it with parse_cfg_valget().
/// @param keys      list of configuration key IDs to query
/// @param layer     which layer to read from (see CfgGetLayer)
/// @param position  pagination offset (0 for the first page)
std::vector<uint8_t> build_cfg_valget(const std::vector<uint32_t>& keys,
                                      uint8_t layer    = CFG_GET_LAYER_RAM,
                                      uint16_t position = 0);

/// Parse the payload of a UBX-CFG-VALGET response frame.
/// @param payload  bytes after the UBX header (i.e. frame[6..N-3])
/// @returns list of key-value pairs extracted from the response
std::vector<CfgVal> parse_cfg_valget(const std::vector<uint8_t>& payload);

} // namespace gnsspp
