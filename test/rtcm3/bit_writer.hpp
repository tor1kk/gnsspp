#pragma once

#include <cstdint>
#include <vector>


/// Test helper: builds bit-packed RTCM3 payloads (MSB-first).
class BitWriter {
public:
    /// Write @p n unsigned bits from @p val (MSB first, 1 ≤ n ≤ 64).
    void write_u(int n, uint64_t val)
    {
        for (int i = n - 1; i >= 0; --i) {
            uint8_t bit = (val >> i) & 1u;
            if (bit_pos_ % 8 == 0) data_.push_back(0);
            data_.back() |= static_cast<uint8_t>(bit << (7 - (bit_pos_ % 8)));
            ++bit_pos_;
        }
    }

    /// Write @p n signed bits from @p val (two's complement, MSB first).
    void write_s(int n, int64_t val)
    {
        // Mask off sign-extension bits beyond n.
        uint64_t mask = (n < 64) ? ((uint64_t(1) << n) - 1) : ~uint64_t(0);
        write_u(n, static_cast<uint64_t>(val) & mask);
    }

    std::vector<uint8_t> data() const { return data_; }

private:
    std::vector<uint8_t> data_;
    int                  bit_pos_ = 0;
};
