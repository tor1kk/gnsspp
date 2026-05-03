#pragma once

#include <string>
#include "gnsspp/port.hpp"


namespace gnsspp {

/// GNSS port backed by a UART/RS-232 device (termios, POSIX only).
///
/// Internally buffers reads: read_byte() and read() are served from a
/// 512-byte userspace buffer, refilled with a single read() syscall.
class PosixSerialPort: public Port {
public:
    /// @param path      device path, e.g. "/dev/ttyUSB0"
    /// @param baudrate  baud rate, e.g. 115200
    PosixSerialPort(const std::string& path, int baudrate);
    ~PosixSerialPort() override;

    void open() override;
    void close() override;
    bool is_open() const override;
    bool wait_readable(int timeout_ms) override;
    uint8_t read_byte() override;
    size_t read(uint8_t* buf, size_t len) override;
    void write(const uint8_t* buf, size_t len) override;

private:
    void refill();

    static constexpr size_t BUF_SIZE = 512;

    std::string path_;
    int         baudrate_;
    int         fd_;

    uint8_t buf_[BUF_SIZE];
    size_t  head_ = 0;
    size_t  tail_ = 0;
};

} // namespace gnsspp
