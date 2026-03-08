#pragma once

#include <cstdint>

#include "gnsspp/port.hpp"
#include "gnsspp/frame.hpp"


namespace gnsspp {

class Parser {
public:
    virtual ~Parser() = default;

    /// Returns true if this parser handles frames starting with @p b1, @p b2.
    /// Both bytes have already been consumed from the port.
    virtual bool matches(uint8_t b1, uint8_t b2) const = 0;

    /// Read and parse a complete frame from @p port.
    /// Both sync bytes @p b1 and @p b2 have already been consumed.
    /// @throws gnsspp::ParseError on malformed frame; gnsspp::IoError on I/O error.
    virtual Frame parse(Port& port, uint8_t b1, uint8_t b2) = 0;
};

} // namespace gnsspp
