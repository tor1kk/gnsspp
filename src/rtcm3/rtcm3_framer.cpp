#include "gnsspp/rtcm3/rtcm3_framer.hpp"


namespace gnsspp {

void Rtcm3Framer::feed(const uint8_t* data, size_t len)
{
    buf_.insert(buf_.end(), data, data + len);
}

std::optional<std::vector<uint8_t>> Rtcm3Framer::next_frame()
{
    while (buf_.size() >= 6) {  // minimum: 3 header + 0 payload + 3 CRC
        // Find preamble with reserved bits check
        if (buf_[0] != 0xD3 || (buf_[1] & 0xFC) != 0x00) {
            buf_.erase(buf_.begin());
            continue;
        }

        uint16_t payload_len = static_cast<uint16_t>((buf_[1] & 0x03u) << 8) | buf_[2];
        size_t   total       = 3u + payload_len + 3u;

        if (buf_.size() < total)
            return std::nullopt;

        // Verify CRC-24Q over header + payload
        uint32_t calc = crc24q(buf_.data(), 3 + payload_len);
        uint32_t recv = (static_cast<uint32_t>(buf_[3 + payload_len])     << 16) |
                        (static_cast<uint32_t>(buf_[3 + payload_len + 1]) <<  8) |
                         static_cast<uint32_t>(buf_[3 + payload_len + 2]);

        if (calc != recv) {
            buf_.erase(buf_.begin());
            continue;
        }

        std::vector<uint8_t> frame(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
        return frame;
    }
    return std::nullopt;
}

uint32_t Rtcm3Framer::crc24q(const uint8_t* data, size_t len)
{
    static constexpr uint32_t POLY = 0x1864CFB;
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int j = 0; j < 8; ++j) {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= POLY;
        }
    }
    return crc & 0xFFFFFFu;
}

} // namespace gnsspp
