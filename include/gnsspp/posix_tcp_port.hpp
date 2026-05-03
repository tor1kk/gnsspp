#pragma once

#include <string>
#include "gnsspp/port.hpp"


namespace gnsspp
{

/// GNSS port backed by a TCP socket (POSIX only).
class PosixTcpPort: public Port {
public:
    /// @param host  hostname or IP address of the TCP server   
    /// @param port  TCP port number
    PosixTcpPort(const std::string& host, int port);
    ~PosixTcpPort() override;   

    void open() override;
    void close() override;
    bool is_open() const override;
    bool wait_readable(int timeout_ms) override;
    uint8_t read_byte() override;
    size_t read(uint8_t* buf, size_t len) override;
    void write(const uint8_t* buf, size_t len) override;

private:
    static constexpr size_t BUF_SIZE = 1024;

    uint8_t buf_[BUF_SIZE];
    size_t  head_ = 0;
    size_t  tail_ = 0;

    std::string host_;
    int         port_;
    int         sockfd_;
};

} // namespace gnsspp
  