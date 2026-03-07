#include "gnsspp/error.hpp"
#include "bit_reader.hpp"


namespace gnsspp {

uint64_t BitReader::read_u64(int n)
{
    if (n < 1 || n > 64)
        throw std::invalid_argument("BitReader::read_u64: n out of range");
    if (remaining() < n)
        throw ParseError("BitReader: not enough bits");

    uint64_t result = 0;
    for (int i = 0; i < n; ++i) {
        // RTCM3 is MSB-first: bit 7 of each byte is the first bit transmitted.
        size_t  byte_idx = bit_pos_ / 8;
        int     bit_idx  = 7 - static_cast<int>(bit_pos_ % 8);
        uint8_t bit      = (data_[byte_idx] >> bit_idx) & 1u;
        result = (result << 1) | bit;
        ++bit_pos_;
    }
    return result;
}


uint32_t BitReader::read_u(int n)
{
    if (n < 1 || n > 32)
        throw std::invalid_argument("BitReader::read_u: n out of range");
    return static_cast<uint32_t>(read_u64(n));
}


int64_t BitReader::read_s64(int n)
{
    uint64_t u = read_u64(n);
    // Sign-extend: if MSB of the n-bit value is 1, extend to full 64 bits.
    if (n < 64 && ((u >> (n - 1)) & 1u)) {
        u |= ~((uint64_t(1) << n) - 1);
    }
    return static_cast<int64_t>(u);
}


int32_t BitReader::read_s(int n)
{
    if (n < 1 || n > 32)
        throw std::invalid_argument("BitReader::read_s: n out of range");
    return static_cast<int32_t>(read_s64(n));
}


void BitReader::skip(int n)
{
    if (remaining() < n)
        throw ParseError("BitReader: not enough bits to skip");
    bit_pos_ += static_cast<size_t>(n);
}

} // namespace gnsspp
