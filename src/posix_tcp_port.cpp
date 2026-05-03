#include <cstring>

#include <errno.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "gnsspp/error.hpp"
#include "gnsspp/posix_tcp_port.hpp"


namespace gnsspp {

PosixTcpPort::PosixTcpPort(const std::string& host, int port)
    : host_(host), port_(port), sockfd_(-1)
{
}


PosixTcpPort::~PosixTcpPort()
{
    if (sockfd_ != -1) {
        int fd = sockfd_;
        sockfd_ = -1;
        ::close(fd);
    }
}


void PosixTcpPort::open(void)
{
    if (sockfd_ != -1)
        throw IoError("port already open");

    struct addrinfo hints{};
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* res = nullptr;
    int err = ::getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res);
    if (err != 0)
        throw IoError(std::string("getaddrinfo: ") + gai_strerror(err));

    sockfd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sockfd_ < 0) {
        ::freeaddrinfo(res);
        throw IoError(std::string("socket: ") + strerror(errno));
    }

    if (::connect(sockfd_, res->ai_addr, res->ai_addrlen) != 0) {
        ::freeaddrinfo(res);
        ::close(sockfd_);
        sockfd_ = -1;
        throw IoError(std::string("connect: ") + strerror(errno));
    }

    ::freeaddrinfo(res);
}


void PosixTcpPort::close(void)
{
    if (sockfd_ == -1) return;
    ::close(sockfd_);
    sockfd_ = -1;
    head_ = tail_ = 0;
}


bool PosixTcpPort::is_open() const
{
    return sockfd_ != -1;
}


bool PosixTcpPort::wait_readable(int timeout_ms)
{
    if (head_ < tail_)
        return true;

    struct pollfd pfd{};
    pfd.fd     = sockfd_;
    pfd.events = POLLIN;

    int ret;
    do {
        ret = poll(&pfd, 1, timeout_ms);
    } while (ret < 0 && errno == EINTR);

    if (ret < 0)
        throw IoError(std::string("poll: ") + strerror(errno));

    return ret > 0;
}


static void refill(int fd, uint8_t* buf, size_t buf_size, size_t& head, size_t& tail)
{
    ssize_t ret = ::recv(fd, buf, buf_size, 0);
    if (ret < 0)
        throw IoError(std::string("recv: ") + strerror(errno));
    if (ret == 0)
        throw IoError("recv: connection closed");
    head = 0;
    tail = static_cast<size_t>(ret);
}


uint8_t PosixTcpPort::read_byte(void)
{
    if (head_ >= tail_)
        refill(sockfd_, buf_, BUF_SIZE, head_, tail_);
    return buf_[head_++];
}


size_t PosixTcpPort::read(uint8_t* buf, size_t len)
{
    if (head_ < tail_) {
        size_t avail = tail_ - head_;
        size_t n = (len < avail) ? len : avail;
        std::memcpy(buf, buf_ + head_, n);
        head_ += n;
        return n;
    }

    ssize_t ret = ::recv(sockfd_, buf, len, 0);
    if (ret < 0)
        throw IoError(std::string("recv: ") + strerror(errno));
    if (ret == 0)
        throw IoError("recv: connection closed");
    return static_cast<size_t>(ret);
}


void PosixTcpPort::write(const uint8_t* buf, size_t len)
{
    while (len > 0) {
        ssize_t n = ::send(sockfd_, buf, len, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw IoError(std::string("send: ") + strerror(errno));
        }
        if (n == 0) {
            throw IoError("send: wrote 0 bytes");
        }
        buf += n;
        len -= n;
    }
}

} // namespace gnsspp
