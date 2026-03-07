#include <cmath>
#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/rtcm3/msm4.hpp"
#include "bit_reader.hpp"


namespace gnsspp {

// Speed of light [m/ms] — for converting rough ranges and fine corrections.
static constexpr double LIGHT_SPEED_M_PER_MS = 299792.458;

// DF400 invalid sentinel: minimum 15-bit signed value.
static constexpr int32_t DF400_INVALID = -(1 << 14);  // -16384

// DF401 invalid sentinel: minimum 22-bit signed value.
static constexpr int32_t DF401_INVALID = -(1 << 21);  // -2097152

// DF397 invalid satellite slot.
static constexpr uint32_t DF397_INVALID = 0xFF;


// MSM4 wire layout:
//
// Fixed header (169 bits):
//   DF002  12  Message Number
//   DF003  12  Reference Station ID
//   Epoch  30  GNSS epoch time (ms, interpretation depends on constellation)
//   DF393   1  Multiple Message Bit
//   DF409   3  IODS
//   ---     7  Reserved
//   DF411   2  Clock Steering Indicator
//   DF412   2  External Clock Indicator
//   DF417   1  GNSS Divergence-Free Smoothing Indicator
//   DF418   3  GNSS Smoothing Interval
//   DF394  64  GNSS Satellite Mask
//   DF395  32  GNSS Signal Mask
//
// Variable header:
//   DF396  nSat*nSig  GNSS Cell Mask
//
// Satellite data (per satellite):
//   DF397   8  Rough Ranges integer ms part  (per sat)
//   DF398  10  Rough Ranges modulo 1 ms      (per sat)
//
// Signal data (per cell with cell_mask == 1):
//   DF400  15  Fine Pseudoranges   (signed, scale 2^-24 ms)
//   DF401  22  Fine PhaseRanges    (signed, scale 2^-29 ms)
//   DF402   4  Lock Time Indicator
//   DF420   1  Half-Cycle Ambiguity Indicator
//   DF403   6  Signal CNR [dBHz]


Msm4 decode_msm4(const std::vector<uint8_t>& payload)
{
    // Minimum: 169-bit fixed header + 1 cell → always > 21 bytes.
    if (payload.size() < 22)
        throw ParseError("RTCM MSM4: payload too short");

    BitReader br(payload);

    // ---- Fixed header ----
    uint32_t msg_type   = br.read_u(12);     // DF002
    uint32_t station_id = br.read_u(12);     // DF003
    uint32_t epoch_ms   = br.read_u(30);     // Epoch time
    bool     multi      = br.read_u(1) != 0; // DF393
    br.skip(3 + 7 + 2 + 2 + 1 + 3);         // DF409, reserved, DF411, DF412, DF417, DF418

    uint64_t sat_mask = br.read_u64(64);     // DF394
    uint32_t sig_mask = br.read_u(32);       // DF395

    // Derive sat/sig ID lists from masks (MSB = ID 1).
    std::vector<uint8_t> sat_ids, sig_ids;
    for (int i = 0; i < 64; ++i)
        if ((sat_mask >> (63 - i)) & 1u) sat_ids.push_back(static_cast<uint8_t>(i + 1));
    for (int i = 0; i < 32; ++i)
        if ((sig_mask >> (31 - i)) & 1u) sig_ids.push_back(static_cast<uint8_t>(i + 1));

    int n_sat = static_cast<int>(sat_ids.size());
    int n_sig = static_cast<int>(sig_ids.size());
    int n_cells = n_sat * n_sig;

    // DF396: cell mask (n_sat * n_sig bits)
    std::vector<bool> cell_mask(static_cast<size_t>(n_cells));
    int n_data_cells = 0;
    for (int i = 0; i < n_cells; ++i) {
        cell_mask[static_cast<size_t>(i)] = (br.read_u(1) != 0);
        if (cell_mask[static_cast<size_t>(i)]) ++n_data_cells;
    }

    // ---- Satellite data ----
    std::vector<uint32_t> df397(static_cast<size_t>(n_sat));
    std::vector<uint32_t> df398(static_cast<size_t>(n_sat));
    for (int i = 0; i < n_sat; ++i) df397[static_cast<size_t>(i)] = br.read_u(8);
    for (int i = 0; i < n_sat; ++i) df398[static_cast<size_t>(i)] = br.read_u(10);

    // ---- Signal data ----
    std::vector<int32_t>  df400(static_cast<size_t>(n_data_cells));
    std::vector<int32_t>  df401(static_cast<size_t>(n_data_cells));
    std::vector<uint32_t> df402(static_cast<size_t>(n_data_cells));
    std::vector<bool>     df420(static_cast<size_t>(n_data_cells));
    std::vector<uint32_t> df403(static_cast<size_t>(n_data_cells));
    for (int i = 0; i < n_data_cells; ++i) df400[static_cast<size_t>(i)] = br.read_s(15);
    for (int i = 0; i < n_data_cells; ++i) df401[static_cast<size_t>(i)] = br.read_s(22);
    for (int i = 0; i < n_data_cells; ++i) df402[static_cast<size_t>(i)] = br.read_u(4);
    for (int i = 0; i < n_data_cells; ++i) df420[static_cast<size_t>(i)] = (br.read_u(1) != 0);
    for (int i = 0; i < n_data_cells; ++i) df403[static_cast<size_t>(i)] = br.read_u(6);

    // ---- Assemble output ----
    Msm4 out{};
    out.msg_type      = static_cast<uint16_t>(msg_type);
    out.station_id    = static_cast<uint16_t>(station_id);
    out.epoch_time_ms = epoch_ms;
    out.multiple_msg  = multi;

    for (int i = 0; i < n_sat; ++i) {
        Msm4Sat sat{};
        sat.id = sat_ids[static_cast<size_t>(i)];
        if (df397[static_cast<size_t>(i)] == DF397_INVALID) {
            sat.rough_range_ms = std::numeric_limits<double>::quiet_NaN();
        } else {
            sat.rough_range_ms = df397[static_cast<size_t>(i)]
                                 + df398[static_cast<size_t>(i)] * (1.0 / 1024.0);
        }
        out.satellites.push_back(sat);
    }

    int cell_idx = 0;
    for (int s = 0; s < n_sat; ++s) {
        for (int si = 0; si < n_sig; ++si) {
            if (!cell_mask[static_cast<size_t>(s * n_sig + si)]) continue;

            Msm4Signal sig{};
            sig.sat_id       = sat_ids[static_cast<size_t>(s)];
            sig.sig_id       = sig_ids[static_cast<size_t>(si)];
            sig.lock_time_ind = static_cast<uint8_t>(df402[static_cast<size_t>(cell_idx)]);
            sig.half_cycle    = df420[static_cast<size_t>(cell_idx)];
            sig.cnr_dbhz      = static_cast<float>(df403[static_cast<size_t>(cell_idx)]);

            double rough_ms = out.satellites[static_cast<size_t>(s)].rough_range_ms;

            // Pseudorange
            int32_t fine_pr = df400[static_cast<size_t>(cell_idx)];
            if (fine_pr == DF400_INVALID || std::isnan(rough_ms)) {
                sig.pseudorange_m = std::numeric_limits<double>::quiet_NaN();
            } else {
                double total_ms = rough_ms + fine_pr * (1.0 / (1 << 24));
                sig.pseudorange_m = total_ms * LIGHT_SPEED_M_PER_MS;
            }

            // Carrier-phase range
            int32_t fine_ph = df401[static_cast<size_t>(cell_idx)];
            if (fine_ph == DF401_INVALID || std::isnan(rough_ms)) {
                sig.phase_range_m = std::numeric_limits<double>::quiet_NaN();
            } else {
                double total_ms = rough_ms + fine_ph * (1.0 / (1 << 29));
                sig.phase_range_m = total_ms * LIGHT_SPEED_M_PER_MS;
            }

            out.signals.push_back(sig);
            ++cell_idx;
        }
    }

    return out;
}

} // namespace gnsspp
