#include <pybind11/pybind11.h>

#include "gnsspp/error.hpp"
#include "gnsspp/frame.hpp"
#include "gnsspp/frame_reader.hpp"
#include "gnsspp/parser.hpp"
#include "gnsspp/port.hpp"
#ifdef _WIN32
#  error "Windows serial port is not yet implemented"
#else
#  include "gnsspp/posix_serial_port.hpp"
   using PlatformSerialPort = gnsspp::PosixSerialPort;
#endif
#include "gnsspp/ubx/ubx_parser.hpp"
#include "gnsspp/nmea/nmea_parser.hpp"
#include "gnsspp/rtcm3/rtcm3_parser.hpp"

#include "helpers.hpp"

namespace py = pybind11;

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

    py::class_<gnsspp::Port>(m, "Port")
        .def("open",          &gnsspp::Port::open)
        .def("close",         &gnsspp::Port::close)
        .def("is_open",       &gnsspp::Port::is_open)
        .def("wait_readable", &gnsspp::Port::wait_readable, py::arg("timeout_ms"))
        .def("read_byte",     &gnsspp::Port::read_byte)
        .def("write", [](gnsspp::Port& self, py::bytes data) {
            std::string s = data;
            return self.write(reinterpret_cast<const uint8_t*>(s.data()), s.size());
        }, py::arg("data"));

    py::class_<PlatformSerialPort, gnsspp::Port>(m, "SerialPort")
        .def(py::init<const std::string&, int>(),
             py::arg("path"), py::arg("baudrate"))
        .def("open",          &PlatformSerialPort::open)
        .def("close",         &PlatformSerialPort::close)
        .def("is_open",       &PlatformSerialPort::is_open)
        .def("wait_readable", &PlatformSerialPort::wait_readable,
             py::arg("timeout_ms"))
        .def("__repr__", [](const PlatformSerialPort&) {
            return "<SerialPort>";
        });

    // unique_ptr holder: pybind11 transfers ownership to FrameReader on add_parser()
    py::class_<gnsspp::Parser, std::unique_ptr<gnsspp::Parser>>(m, "Parser");

    py::class_<gnsspp::UBXParser, gnsspp::Parser,
               std::unique_ptr<gnsspp::UBXParser>>(m, "UBXParser")
        .def(py::init<>());

    py::class_<gnsspp::NMEAParser, gnsspp::Parser,
               std::unique_ptr<gnsspp::NMEAParser>>(m, "NMEAParser")
        .def(py::init<>());

    py::class_<gnsspp::RTCM3Parser, gnsspp::Parser,
               std::unique_ptr<gnsspp::RTCM3Parser>>(m, "RTCM3Parser")
        .def(py::init<>());

    // keep_alive<1,2>: port (arg2) is kept alive as long as FrameReader (arg1=self)
    py::class_<gnsspp::FrameReader>(m, "FrameReader")
        .def(py::init<gnsspp::Port&>(), py::keep_alive<1, 2>(), py::arg("port"))
        .def("add_parser", &gnsspp::FrameReader::add_parser, py::arg("parser"))
        .def("read_frame", &gnsspp::FrameReader::read_frame,
             py::arg("timeout_ms") = -1);
}
