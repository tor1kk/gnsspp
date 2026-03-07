#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "gnsspp/port.hpp"
#include "gnsspp/frame.hpp"
#include "gnsspp/iparser.hpp"


namespace gnsspp {

/// Reads one complete frame of any supported protocol from a Port.
/// Parsers are registered with add_parser(); the first one whose matches()
/// returns true for the preamble byte handles the frame.
class FrameReader {
public:
    explicit FrameReader(Port& port);

    /// Register a parser for a specific protocol (UBX, NMEA, RTCM, ...).
    void add_parser(std::unique_ptr<IParser> parser);

    /// Read one complete frame from the port.
    /// @param timeout_ms  max wait for the first (preamble) byte; -1 = forever.
    /// @return parsed frame, or std::nullopt on timeout.
    /// @throws gnsspp::IoError on I/O error; gnsspp::ParseError on unrecognised preamble.
    std::optional<Frame> read_frame(int timeout_ms = -1);

private:
    Port& port_;
    std::vector<std::unique_ptr<IParser>> parsers_;
};

} // namespace gnsspp
