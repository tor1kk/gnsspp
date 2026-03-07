#pragma once

#include <cstdint>
#include <string>
#include <vector>


namespace gnsspp {

/// A single GNSS protocol frame as received from a port.
struct Frame {
    std::vector<uint8_t> raw;           ///< Raw bytes (header + payload + checksum).
    std::string          protocol;      ///< "UBX", "NMEA", "RTCM3"
    std::string          type;          ///< "NAV-PVT", "NAV-SAT", "GGA", ...
    uint16_t             payload_offset = 0; ///< Byte offset of payload start within raw.
    uint16_t             payload_size   = 0; ///< Payload length in bytes.

    /// Returns a copy of the payload bytes (protocol header/checksum stripped).
    std::vector<uint8_t> payload() const
    {
        return {raw.begin() + payload_offset,
                raw.begin() + payload_offset + payload_size};
    }
};

} // namespace gnsspp
