#pragma once

#include <stdexcept>
#include <string>


namespace gnsspp {

/// Base class for all gnsspp exceptions.
class Error : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

/// Thrown when an I/O operation fails: port closed unexpectedly, OS error,
/// or timeout while waiting for data.
class IoError : public Error {
    using Error::Error;
};

/// Thrown when a received frame is malformed: bad checksum/CRC, payload too
/// short, too few fields, or unrecognised message type.
class ParseError : public Error {
    using Error::Error;
};

} // namespace gnsspp
