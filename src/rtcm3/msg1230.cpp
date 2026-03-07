#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/rtcm3/msg1230.hpp"
#include "bit_reader.hpp"


namespace gnsspp {

// RTCM 1230 wire layout (variable):
//
//  DF002  12  Message Number           (= 1230)
//  DF003  12  Reference Station ID
//  DF421   1  GLONASS Code-Phase Bias Indicator
//  ---     3  Reserved
//  DF422   4  GLONASS FDMA Signal Mask
//             bit 3 (MSB) = L1 C/A present
//             bit 2       = L1 P  present
//             bit 1       = L2 C/A present
//             bit 0 (LSB) = L2 P  present
//
//  For each set bit in DF422 (in MSB→LSB order):
//  DF423  16  L1 C/A Code-Phase Bias  (signed, scale 0.02 m)
//  DF424  16  L1 P  Code-Phase Bias   (signed, scale 0.02 m)
//  DF425  16  L2 C/A Code-Phase Bias  (signed, scale 0.02 m)
//  DF426  16  L2 P  Code-Phase Bias   (signed, scale 0.02 m)
//
// Minimum payload: (12+12+1+3+4) bits = 32 bits = 4 bytes.


Msg1230 decode_msg1230(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 4)
        throw ParseError("RTCM 1230: payload too short");

    BitReader br(payload);

    br.skip(12);
    uint32_t station_id   = br.read_u(12);   // DF003
    bool     bias_ind     = br.read_u(1) != 0; // DF421
    br.skip(3);                               // reserved
    uint32_t sig_mask     = br.read_u(4);    // DF422

    bool has_l1ca = (sig_mask >> 3) & 1u;
    bool has_l1p  = (sig_mask >> 2) & 1u;
    bool has_l2ca = (sig_mask >> 1) & 1u;
    bool has_l2p  =  sig_mask       & 1u;

    // Each present bias field is 16 bits, signed, scale 0.02 m.
    int required_bits = (has_l1ca + has_l1p + has_l2ca + has_l2p) * 16;
    if (br.remaining() < required_bits)
        throw ParseError("RTCM 1230: payload too short for declared biases");

    auto read_bias = [&](bool present) -> Msg1230::Bias {
        if (!present) return {false, 0.0};
        return {true, br.read_s(16) * 0.02};
    };

    Msg1230 out{};
    out.station_id    = static_cast<uint16_t>(station_id);
    out.bias_indicator = bias_ind;
    out.l1ca = read_bias(has_l1ca);
    out.l1p  = read_bias(has_l1p);
    out.l2ca = read_bias(has_l2ca);
    out.l2p  = read_bias(has_l2p);

    return out;
}

} // namespace gnsspp
