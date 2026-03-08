#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>


namespace gnsspp {

/// Bit-level reader for RTCM 3.x messages (MSB-first bit ordering).
/// All read functions advance the internal bit position.
class BitReader {
public:
    explicit BitReader(const std::vector<uint8_t>& data)
        : data_(data), bit_pos_(0) {}

    /// Read @p n unsigned bits (1 ≤ n ≤ 32).
    uint32_t read_u(int n);

    /// Read @p n unsigned bits (1 ≤ n ≤ 64).
    uint64_t read_u64(int n);

    /// Read @p n signed bits, sign-extended to int32 (1 ≤ n ≤ 32).
    int32_t read_s(int n);

    /// Read @p n signed bits, sign-extended to int64 (1 ≤ n ≤ 64).
    int64_t read_s64(int n);

    /// Skip @p n bits.
    void skip(int n);

    /// Remaining bits in the buffer.
    int remaining() const
    {
        return static_cast<int>(data_.size() * 8) - static_cast<int>(bit_pos_);
    }

private:
    const std::vector<uint8_t> data_;
    size_t                       bit_pos_;
};

} // namespace gnsspp
