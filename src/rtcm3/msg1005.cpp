#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/rtcm3/msg1005.hpp"
#include "bit_reader.hpp"


namespace gnsspp {

// RTCM 1005 wire layout (152 bits = 19 bytes):
//
//  DF002  12  Message Number           (= 1005)
//  DF003  12  Reference Station ID
//  DF021   6  ITRF Realization Year    (ignored)
//  DF022   1  GPS Indicator
//  DF023   1  GLONASS Indicator
//  DF024   1  Galileo Indicator
//  DF141   1  Reference-Station Indicator (ignored)
//  DF025  38  Antenna Ref. Point ECEF-X  (signed, scale 0.0001 m)
//  DF142   1  Single Receiver Oscillator Indicator (ignored)
//  DF001   1  Reserved                  (ignored)
//  DF026  38  Antenna Ref. Point ECEF-Y  (signed, scale 0.0001 m)
//  DF364   2  Quarter Cycle Indicator    (ignored)
//  DF027  38  Antenna Ref. Point ECEF-Z  (signed, scale 0.0001 m)


Msg1005 decode_msg1005(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 19)
        throw ParseError("RTCM 1005: payload too short");

    BitReader br(payload);

    br.skip(12);                                // DF002 message number
    uint32_t station_id = br.read_u(12);        // DF003
    br.skip(6);                                 // DF021 ITRF year
    bool gps     = br.read_u(1) != 0;           // DF022
    bool glonass = br.read_u(1) != 0;           // DF023
    bool galileo = br.read_u(1) != 0;           // DF024
    br.skip(1);                                 // DF141
    int64_t x_raw = br.read_s64(38);            // DF025
    br.skip(2);                                 // DF142 + DF001
    int64_t y_raw = br.read_s64(38);            // DF026
    br.skip(2);                                 // DF364
    int64_t z_raw = br.read_s64(38);            // DF027

    Msg1005 out{};
    out.station_id = static_cast<uint16_t>(station_id);
    out.gps        = gps;
    out.glonass    = glonass;
    out.galileo    = galileo;
    out.ecef_x     = x_raw * 0.0001;
    out.ecef_y     = y_raw * 0.0001;
    out.ecef_z     = z_raw * 0.0001;

    return out;
}

} // namespace gnsspp
