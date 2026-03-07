#pragma once

#include <string>
#include "gnsspp/ports/port.hpp"


namespace gnsspp {

/// GNSS port backed by a UART/RS-232 device (termios).
class SerialPort: public Port {
public:
    /// @param path      device path, e.g. "/dev/ttyUSB0"
    /// @param baudrate  baud rate, e.g. 115200
    SerialPort(const std::string& path, int baudrate);
    ~SerialPort() override;

    void open() override;
    void close() override;
    bool is_open() const override;
    bool wait_readable(int timeout_ms) override;
    uint8_t read_byte() override;
    size_t read(uint8_t* buf, size_t len) override;

private:
    std::string path_;
    int baudrate_;
    int fd_;
};

} // namespace gnsspp
