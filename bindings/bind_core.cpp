#include <algorithm>
#include <cstring>

#include <pybind11/pybind11.h>

#include "gnsspp/error.hpp"
#include "gnsspp/frame.hpp"
#include "gnsspp/frame_reader.hpp"
#include "gnsspp/parser.hpp"
#include "gnsspp/port.hpp"
#include "gnsspp/ubx/ubx_parser.hpp"
#include "gnsspp/nmea/nmea_parser.hpp"
#include "gnsspp/rtcm3/rtcm3_parser.hpp"

#include "helpers.hpp"

namespace py = pybind11;

// ---------------------------------------------------------------------------
// PyPort — trampoline that lets Python subclass the abstract Port interface.
// Python must implement: open, close, is_open, wait_readable, read_byte, write.
// The read(n) override is optional; default calls read_byte() n times.
// ---------------------------------------------------------------------------

class PyPort : public gnsspp::Port {
public:
    void open() override
    {
        PYBIND11_OVERRIDE_PURE(void, gnsspp::Port, open);
    }

    void close() override
    {
        PYBIND11_OVERRIDE_PURE(void, gnsspp::Port, close);
    }

    bool is_open() const override
    {
        PYBIND11_OVERRIDE_PURE(bool, gnsspp::Port, is_open);
    }

    bool wait_readable(int timeout_ms) override
    {
        PYBIND11_OVERRIDE_PURE(bool, gnsspp::Port, wait_readable, timeout_ms);
    }

    uint8_t read_byte() override
    {
        PYBIND11_OVERRIDE_PURE(uint8_t, gnsspp::Port, read_byte);
    }

    size_t read(uint8_t* buf, size_t len) override
    {
        // Python override receives requested length, must return bytes.
        py::gil_scoped_acquire gil;
        auto override = py::get_override(this, "read");
        if (!override) {
            // Default: call read_byte() len times.
            for (size_t i = 0; i < len; ++i)
                buf[i] = read_byte();
            return len;
        }
        py::bytes result = override(len).cast<py::bytes>();
        std::string s = result;
        size_t n = std::min(s.size(), len);
        std::memcpy(buf, s.data(), n);
        return n;
    }

    size_t write(const uint8_t* buf, size_t len) override
    {
        py::gil_scoped_acquire gil;
        auto override = py::get_override(this, "write");
        if (!override)
            throw std::runtime_error("PyPort.write() not implemented");
        py::object result = override(
            py::bytes(reinterpret_cast<const char*>(buf), len));
        return result.cast<size_t>();
    }
};

// ---------------------------------------------------------------------------

void bind_core(py::module_& m)
{
    py::register_exception<gnsspp::IoError>(m, "IoError");
    py::register_exception<gnsspp::ParseError>(m, "ParseError");

    py::class_<gnsspp::Frame>(m, "Frame")
        .def_readonly("protocol",       &gnsspp::Frame::protocol)
        .def_readonly("type",           &gnsspp::Frame::type)
        .def_readonly("payload_offset", &gnsspp::Frame::payload_offset)
        .def_readonly("payload_size",   &gnsspp::Frame::payload_size)
        .def_property_readonly("raw", [](const gnsspp::Frame& f) {
            return vec_to_bytes(f.raw);
        })
        .def("payload", [](const gnsspp::Frame& f) {
            return vec_to_bytes(f.payload());
        })
        .def("__repr__", [](const gnsspp::Frame& f) {
            return "<Frame protocol=" + f.protocol + " type=" + f.type + ">";
        });

    // Port base — exposed so Python can subclass it.
    py::class_<gnsspp::Port, PyPort>(m, "Port")
        .def(py::init<>())
        .def("open",          &gnsspp::Port::open)
        .def("close",         &gnsspp::Port::close)
        .def("is_open",       &gnsspp::Port::is_open)
        .def("wait_readable", &gnsspp::Port::wait_readable, py::arg("timeout_ms"))
        .def("read_byte",     &gnsspp::Port::read_byte)
        .def("write", [](gnsspp::Port& self, py::bytes data) {
            std::string s = data;
            return self.write(reinterpret_cast<const uint8_t*>(s.data()), s.size());
        }, py::arg("data"));

    // smart_holder enables unique_ptr ownership transfer from Python to C++
    // (required by pybind11 v3 for add_parser()).
    py::class_<gnsspp::Parser, py::smart_holder>(m, "Parser");

    py::class_<gnsspp::UBXParser, gnsspp::Parser, py::smart_holder>(m, "UBXParser")
        .def(py::init<>());

    py::class_<gnsspp::NMEAParser, gnsspp::Parser, py::smart_holder>(m, "NMEAParser")
        .def(py::init<>());

    py::class_<gnsspp::RTCM3Parser, gnsspp::Parser, py::smart_holder>(m, "RTCM3Parser")
        .def(py::init<>());

    // keep_alive<1,2>: port (arg2) is kept alive as long as FrameReader (arg1=self)
    // GIL is released during read_frame() so Python threads can run while blocked.
    py::class_<gnsspp::FrameReader>(m, "FrameReader")
        .def(py::init<gnsspp::Port&>(), py::keep_alive<1, 2>(), py::arg("port"))
        .def("add_parser", &gnsspp::FrameReader::add_parser, py::arg("parser"))
        .def("read_frame",
             [](gnsspp::FrameReader& self, int timeout_ms) -> py::object {
                 std::optional<gnsspp::Frame> f;
                 {
                     py::gil_scoped_release release;
                     f = self.read_frame(timeout_ms);
                 }
                 if (!f) return py::none();
                 return py::cast(std::move(*f));
             },
             py::arg("timeout_ms") = -1);
}
