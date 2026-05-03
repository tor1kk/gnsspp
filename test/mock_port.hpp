#pragma once

#include <vector>
#include <stdexcept>

#include "gnsspp/port.hpp"


class MockPort : public gnsspp::Port {
public:
    MockPort(std::vector<uint8_t> bytes) : buffer_(bytes) {}
    ~MockPort() override = default;

    void open() override {}
    void close() override {}
    bool is_open() const override {return true;}
    bool wait_readable(int timeout_ms) override {
        (void)timeout_ms;
        return buffer_.size() > 0;
    }

    uint8_t read_byte() override {
        if (buffer_.empty()) {
            throw std::runtime_error("read_byte: buffer empty");
        }

        auto tmp = buffer_.front();
        buffer_.erase(buffer_.begin());
        return tmp;
    }

    size_t read(uint8_t* buf, size_t len) override {
        size_t counter = 0;

        for (; counter < len; counter++) {
            if (buffer_.empty()) {
                return counter;
            }

            buf[counter] = buffer_.front();
            buffer_.erase(buffer_.begin());
        }

        return counter;
    }

    void write(const uint8_t* buf, size_t len) override {
        written_.insert(written_.end(), buf, buf + len);
    }

    const std::vector<uint8_t>& written() const { return written_; }

private:
   std::vector<uint8_t> buffer_;
   std::vector<uint8_t> written_;
};
