#include <stdexcept>
#include <cstring>

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>

#include "gnsspp/error.hpp"
#include "gnsspp/posix_serial_port.hpp"


namespace gnsspp {
namespace {

speed_t to_baud(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 38400:  return B38400;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default: throw std::invalid_argument("unsupported baud rate");
    }
}

} // static namespace


PosixSerialPort::PosixSerialPort(const std::string& path, int baudrate)
    : path_(path), baudrate_(baudrate), fd_(-1)
{

}


PosixSerialPort::~PosixSerialPort()
{
    if (fd_ != -1) {
        ::close(fd_);
    }
}


void PosixSerialPort::open(void)
{
    if (fd_ != -1)
        throw IoError("port already open");

    fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY);

    if (fd_ < 0) {
        throw IoError(std::string("open failed: ") + strerror(errno));
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        throw IoError(std::string("tcgetattr: ") + strerror(errno));
    }

    speed_t speed = to_baud(baudrate_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~PARENB;       // No parity
    tty.c_cflag &= ~CSTOPB;       // 1 stop-bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;          // 8 bit packets
    tty.c_cflag |=  CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // raw mode
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // no flow control
    tty.c_oflag &= ~OPOST;                           // no output proc

    tty.c_cc[VMIN]  = 1;   // wait at least 1 byte
    tty.c_cc[VTIME] = 0;   // no timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        throw IoError(std::string("tcsetattr: ") + strerror(errno));
    }

    tcflush(fd_, TCIOFLUSH);
}


void PosixSerialPort::close(void)
{
    if (fd_ == -1) return;
    ::close(fd_);
    fd_ = -1;
}


bool PosixSerialPort::is_open(void) const
{
    return fd_ != -1;
}


bool PosixSerialPort::wait_readable(int timeout_ms)
{
    struct pollfd pfd{};
    pfd.fd     = fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0)
        throw IoError(std::string("poll: ") + strerror(errno));
    return ret > 0;
}


uint8_t PosixSerialPort::read_byte(void)
{
    uint8_t tmp = 0;

    int ret = ::read(fd_, &tmp, 1);
    if (ret < 0)
        throw IoError(std::string("read failed: ") + strerror(errno));
    if (ret == 0)
        throw IoError("read failed: EOF");

    return tmp;
}


size_t PosixSerialPort::read(uint8_t* buf, size_t len)
{
    int ret = ::read(fd_, buf, len);
    if (ret < 0) {
        throw IoError(std::string("read failed: ") + strerror(errno));
    }

    return static_cast<size_t>(ret);
}


size_t PosixSerialPort::write(const uint8_t* buf, size_t len)
{
    int ret = ::write(fd_, buf, len);
    if (ret < 0) {
        throw IoError(std::string("write failed: ") + strerror(errno));
    }

    return static_cast<size_t>(ret);
}

} // namespace gnsspp
