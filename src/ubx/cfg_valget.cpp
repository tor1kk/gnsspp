#include "gnsspp/ubx/cfg_valget.hpp"
#include "ubx_frame.hpp"


namespace gnsspp {

// CFG-VALGET: class 0x06, id 0x8B
static constexpr uint8_t CLASS_CFG      = 0x06;
static constexpr uint8_t ID_CFG_VALGET  = 0x8B;


std::vector<uint8_t> build_cfg_valget(const std::vector<uint32_t>& keys,
                                      uint8_t layer,
                                      uint16_t position)
{
    // Payload: version(1) + layer(1) + position(2) + keys (4 bytes each)
    std::vector<uint8_t> payload;
    payload.reserve(4 + keys.size() * 4);

    payload.push_back(0x00);                      // version
    payload.push_back(layer);
    payload.push_back( position       & 0xFF);    // position lo
    payload.push_back((position >> 8) & 0xFF);    // position hi

    for (const uint32_t key : keys) {
        payload.push_back( key        & 0xFF);
        payload.push_back((key >>  8) & 0xFF);
        payload.push_back((key >> 16) & 0xFF);
        payload.push_back((key >> 24) & 0xFF);
    }

    return detail::build_ubx_frame(CLASS_CFG, ID_CFG_VALGET, payload);
}


std::vector<CfgVal> parse_cfg_valget(const std::vector<uint8_t>& payload)
{
    // Response payload: version(1) + layer(1) + position(2) + key-value pairs
    std::vector<CfgVal> result;

    if (payload.size() < 4)
        return result;

    size_t offset = 4;  // skip header
    while (offset + 4 <= payload.size()) {
        const uint32_t key = static_cast<uint32_t>(payload[offset])
                           | (static_cast<uint32_t>(payload[offset + 1]) <<  8)
                           | (static_cast<uint32_t>(payload[offset + 2]) << 16)
                           | (static_cast<uint32_t>(payload[offset + 3]) << 24);
        offset += 4;

        const size_t val_size = detail::cfg_key_size(key);
        if (val_size == 0 || offset + val_size > payload.size())
            break;

        uint64_t value = 0;
        for (size_t i = 0; i < val_size; ++i)
            value |= static_cast<uint64_t>(payload[offset + i]) << (i * 8);
        offset += val_size;

        result.push_back({key, value});
    }

    return result;
}

} // namespace gnsspp
