#pragma once

#include <cstdint>
#include <cstddef>


namespace gnsspp {

/// Abstract interface for a GNSS data port (serial, mock, etc.).
class Port {
public:
    virtual ~Port() = default;

    Port(const Port&) = delete;
    Port& operator=(const Port&) = delete;

    virtual void open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;

    /// Read exactly one byte. Throws std::runtime_error on failure.
    virtual uint8_t read_byte() = 0;

    /// Read up to @p len bytes into @p buf. Returns number of bytes read.
    virtual size_t read(uint8_t* buf, size_t len) = 0;

    /// Block until data is available or timeout expires.
    /// @param timeout_ms  milliseconds to wait; -1 = wait forever.
    /// @return true if data is ready, false on timeout.
    virtual bool wait_readable(int timeout_ms) = 0;

protected:
    Port() = default;
};

} // namespace gnsspp
