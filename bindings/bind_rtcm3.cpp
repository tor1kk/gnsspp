#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "gnsspp/rtcm3/msm4.hpp"
#include "gnsspp/rtcm3/msg1005.hpp"
#include "gnsspp/rtcm3/msg1006.hpp"
#include "gnsspp/rtcm3/msg1230.hpp"

#include "helpers.hpp"

namespace py = pybind11;

void bind_rtcm3(py::module_& m)
{
    py::class_<gnsspp::Msm4Sat>(m, "Msm4Sat")
        .def_readonly("id",             &gnsspp::Msm4Sat::id)
        .def_readonly("rough_range_ms", &gnsspp::Msm4Sat::rough_range_ms);

    py::class_<gnsspp::Msm4Signal>(m, "Msm4Signal")
        .def_readonly("sat_id",        &gnsspp::Msm4Signal::sat_id)
        .def_readonly("sig_id",        &gnsspp::Msm4Signal::sig_id)
        .def_readonly("pseudorange_m", &gnsspp::Msm4Signal::pseudorange_m)
        .def_readonly("phase_range_m", &gnsspp::Msm4Signal::phase_range_m)
        .def_readonly("lock_time_ind", &gnsspp::Msm4Signal::lock_time_ind)
        .def_readonly("half_cycle",    &gnsspp::Msm4Signal::half_cycle)
        .def_readonly("cnr_dbhz",      &gnsspp::Msm4Signal::cnr_dbhz);

    py::class_<gnsspp::Msm4>(m, "Msm4")
        .def_readonly("msg_type",      &gnsspp::Msm4::msg_type)
        .def_readonly("station_id",    &gnsspp::Msm4::station_id)
        .def_readonly("epoch_time_ms", &gnsspp::Msm4::epoch_time_ms)
        .def_readonly("multiple_msg",  &gnsspp::Msm4::multiple_msg)
        .def_readonly("satellites",    &gnsspp::Msm4::satellites)
        .def_readonly("signals",       &gnsspp::Msm4::signals);

    m.def("decode_msm4", [](py::bytes b) {
        return gnsspp::decode_msm4(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::Msg1005>(m, "Msg1005")
        .def_readonly("station_id", &gnsspp::Msg1005::station_id)
        .def_readonly("gps",        &gnsspp::Msg1005::gps)
        .def_readonly("glonass",    &gnsspp::Msg1005::glonass)
        .def_readonly("galileo",    &gnsspp::Msg1005::galileo)
        .def_readonly("ecef_x",     &gnsspp::Msg1005::ecef_x)
        .def_readonly("ecef_y",     &gnsspp::Msg1005::ecef_y)
        .def_readonly("ecef_z",     &gnsspp::Msg1005::ecef_z);

    m.def("decode_msg1005", [](py::bytes b) {
        return gnsspp::decode_msg1005(bytes_to_vec(b));
    }, py::arg("payload"));

    // Msg1006 inherits Msg1005 — pybind11 inherits all parent bindings
    py::class_<gnsspp::Msg1006, gnsspp::Msg1005>(m, "Msg1006")
        .def_readonly("antenna_height", &gnsspp::Msg1006::antenna_height);

    m.def("decode_msg1006", [](py::bytes b) {
        return gnsspp::decode_msg1006(bytes_to_vec(b));
    }, py::arg("payload"));

    // Nested struct Msg1230::Bias exposed as standalone Msg1230Bias
    py::class_<gnsspp::Msg1230::Bias>(m, "Msg1230Bias")
        .def_readonly("valid",   &gnsspp::Msg1230::Bias::valid)
        .def_readonly("value_m", &gnsspp::Msg1230::Bias::value_m);

    py::class_<gnsspp::Msg1230>(m, "Msg1230")
        .def_readonly("station_id",     &gnsspp::Msg1230::station_id)
        .def_readonly("bias_indicator", &gnsspp::Msg1230::bias_indicator)
        .def_readonly("l1ca",           &gnsspp::Msg1230::l1ca)
        .def_readonly("l1p",            &gnsspp::Msg1230::l1p)
        .def_readonly("l2ca",           &gnsspp::Msg1230::l2ca)
        .def_readonly("l2p",            &gnsspp::Msg1230::l2p);

    m.def("decode_msg1230", [](py::bytes b) {
        return gnsspp::decode_msg1230(bytes_to_vec(b));
    }, py::arg("payload"));
}
