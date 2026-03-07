#include "gnsspp/ubx/cfg_valset.hpp"
#include "ubx_frame.hpp"


namespace gnsspp {

// CFG-VALSET: class 0x06, id 0x8A
static constexpr uint8_t CLASS_CFG    = 0x06;
static constexpr uint8_t ID_CFG_VALSET = 0x8A;


std::vector<uint8_t> build_cfg_valset(const std::vector<CfgVal>& items,
                                      uint8_t layers)
{
    // Payload: version(1) + layers(1) + reserved(2) + key-value pairs
    std::vector<uint8_t> payload;
    payload.reserve(4 + items.size() * 8);

    payload.push_back(0x00);    // version
    payload.push_back(layers);
    payload.push_back(0x00);    // reserved
    payload.push_back(0x00);

    for (const auto& item : items) {
        // Key — 4 bytes little-endian
        payload.push_back( item.key        & 0xFF);
        payload.push_back((item.key >>  8) & 0xFF);
        payload.push_back((item.key >> 16) & 0xFF);
        payload.push_back((item.key >> 24) & 0xFF);

        // Value — N bytes little-endian, size from key bits 28-30
        const size_t val_size = detail::cfg_key_size(item.key);
        for (size_t i = 0; i < val_size; ++i) {
            payload.push_back((item.value >> (i * 8)) & 0xFF);
        }
    }

    return detail::build_ubx_frame(CLASS_CFG, ID_CFG_VALSET, payload);
}

} // namespace gnsspp
