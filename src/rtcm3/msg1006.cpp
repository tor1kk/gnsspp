#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/rtcm3/msg1006.hpp"
#include "bit_reader.hpp"


namespace gnsspp {

// RTCM 1006 wire layout (168 bits = 21 bytes):
// Same as 1005 (152 bits) plus:
//  DF028  16  Antenna Height  (unsigned, scale 0.0001 m)


Msg1006 decode_msg1006(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 21)
        throw ParseError("RTCM 1006: payload too short");

    BitReader br(payload);

    br.skip(12);
    uint32_t station_id = br.read_u(12);
    br.skip(6);
    bool gps     = br.read_u(1) != 0;
    bool glonass = br.read_u(1) != 0;
    bool galileo = br.read_u(1) != 0;
    br.skip(1);
    int64_t x_raw = br.read_s64(38);
    br.skip(2);
    int64_t y_raw = br.read_s64(38);
    br.skip(2);
    int64_t z_raw = br.read_s64(38);
    uint32_t h_raw = br.read_u(16);             // DF028 antenna height

    Msg1006 out{};
    out.station_id     = static_cast<uint16_t>(station_id);
    out.gps            = gps;
    out.glonass        = glonass;
    out.galileo        = galileo;
    out.ecef_x         = x_raw * 0.0001;
    out.ecef_y         = y_raw * 0.0001;
    out.ecef_z         = z_raw * 0.0001;
    out.antenna_height = h_raw * 0.0001;

    return out;
}

} // namespace gnsspp
