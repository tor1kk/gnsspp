#include <cstdio>
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
    if (!port_.wait_readable(timeout_ms)) {
        return std::nullopt;
    }

    uint8_t b1 = port_.read_byte();

    // Sliding-window sync search: advance one byte at a time until a parser
    // recognises the (b1, b2) pair.  This handles leading garbage bytes
    // (e.g. a stray '\n' before a '$' in an NMEA stream) without losing sync.
    for (;;) {
        if (!port_.wait_readable(timeout_ms)) {
            throw IoError("timeout during sync search");
        }
        uint8_t b2 = port_.read_byte();

        for (auto& parser : parsers_) {
            if (parser->matches(b1, b2)) {
                return parser->parse(port_, b1, b2);
            }
        }

        b1 = b2; // slide: discard b1, try again with b2 as the new b1
    }
}

} // namespace gnsspp
