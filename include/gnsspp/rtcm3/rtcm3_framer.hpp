#pragma once

#include <cstdint>
#include <optional>
#include <vector>


namespace gnsspp {

/// Stateful buffer-based RTCM3 frame extractor.
/// Feed raw bytes with feed(); retrieve complete, CRC-verified frames
/// with next_frame(). Analogous to UbxFramer for RTCM3.
class Rtcm3Framer {
public:
    void feed(const uint8_t* data, size_t len);

    /// Returns the next complete RTCM3 frame (raw bytes) or nullopt.
    std::optional<std::vector<uint8_t>> next_frame();

private:
    static uint32_t crc24q(const uint8_t* data, size_t len);

    std::vector<uint8_t> buf_;
};

} // namespace gnsspp
