#include <chrono>
#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/frame_reader.hpp"


namespace gnsspp {

FrameReader::FrameReader(Port& port)
    : port_(port)
{
}


void FrameReader::add_parser(std::unique_ptr<Parser> parser)
{
    parsers_.push_back(std::move(parser));
}


std::optional<Frame> FrameReader::read_frame(int timeout_ms)
{
    using clock = std::chrono::steady_clock;
    using ms    = std::chrono::milliseconds;

    const auto deadline = (timeout_ms < 0)
        ? clock::time_point::max()
        : clock::now() + ms(timeout_ms);

    auto ms_left = [&]() -> int {
        if (timeout_ms < 0) return -1;
        auto rem = std::chrono::duration_cast<ms>(deadline - clock::now()).count();
        return (rem > 0) ? static_cast<int>(rem) : 0;
    };

    if (!port_.wait_readable(ms_left()))
        return std::nullopt;

    uint8_t b1 = port_.read_byte();

    // Sliding-window sync search: advance one byte at a time until a parser
    // recognises the (b1, b2) pair.  This handles leading garbage bytes
    // (e.g. a stray '\n' before a '$' in an NMEA stream) without losing sync.
    for (;;) {
        int left = ms_left();
        if (left == 0)
            return std::nullopt;

        if (!port_.wait_readable(left))
            return std::nullopt;

        uint8_t b2 = port_.read_byte();

        for (auto& parser : parsers_) {
            if (parser->matches(b1, b2))
                return parser->parse(port_, b1, b2);
        }

        b1 = b2; // slide: discard b1, try again with b2 as the new b1
    }
}

} // namespace gnsspp
