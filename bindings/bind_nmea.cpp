#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "gnsspp/nmea/gga.hpp"
#include "gnsspp/nmea/rmc.hpp"
#include "gnsspp/nmea/gsa.hpp"
#include "gnsspp/nmea/gsv.hpp"
#include "gnsspp/nmea/vtg.hpp"

namespace py = pybind11;

void bind_nmea(py::module_& m)
{
    py::class_<gnsspp::NmeaGga>(m, "NmeaGga")
        .def_readonly("utc",         &gnsspp::NmeaGga::utc)
        .def_readonly("lat",         &gnsspp::NmeaGga::lat)
        .def_readonly("lon",         &gnsspp::NmeaGga::lon)
        .def_readonly("fix_quality", &gnsspp::NmeaGga::fix_quality)
        .def_readonly("num_sv",      &gnsspp::NmeaGga::num_sv)
        .def_readonly("hdop",        &gnsspp::NmeaGga::hdop)
        .def_readonly("alt_m",       &gnsspp::NmeaGga::alt_m)
        .def_readonly("geoid_sep_m", &gnsspp::NmeaGga::geoid_sep_m);

    m.def("decode_gga", &gnsspp::decode_gga, py::arg("sentence"));

    py::class_<gnsspp::NmeaRmc>(m, "NmeaRmc")
        .def_readonly("utc",         &gnsspp::NmeaRmc::utc)
        .def_readonly("active",      &gnsspp::NmeaRmc::active)
        .def_readonly("lat",         &gnsspp::NmeaRmc::lat)
        .def_readonly("lon",         &gnsspp::NmeaRmc::lon)
        .def_readonly("speed_knots", &gnsspp::NmeaRmc::speed_knots)
        .def_readonly("course_deg",  &gnsspp::NmeaRmc::course_deg)
        .def_readonly("date",        &gnsspp::NmeaRmc::date);

    m.def("decode_rmc", &gnsspp::decode_rmc, py::arg("sentence"));

    py::class_<gnsspp::NmeaGsa>(m, "NmeaGsa")
        .def_readonly("auto_mode",  &gnsspp::NmeaGsa::auto_mode)
        .def_readonly("fix_type",   &gnsspp::NmeaGsa::fix_type)
        .def_readonly("sv_count",   &gnsspp::NmeaGsa::sv_count)
        .def_readonly("pdop",       &gnsspp::NmeaGsa::pdop)
        .def_readonly("hdop",       &gnsspp::NmeaGsa::hdop)
        .def_readonly("vdop",       &gnsspp::NmeaGsa::vdop)
        .def_readonly("system_id",  &gnsspp::NmeaGsa::system_id)
        // sv_prns is a C array — expose as list of non-zero entries
        .def_property_readonly("sv_prns", [](const gnsspp::NmeaGsa& g) {
            return std::vector<uint8_t>(g.sv_prns, g.sv_prns + g.sv_count);
        });

    m.def("decode_gsa", &gnsspp::decode_gsa, py::arg("sentence"));

    py::class_<gnsspp::GsvSatellite>(m, "GsvSatellite")
        .def_readonly("prn",  &gnsspp::GsvSatellite::prn)
        .def_readonly("elev", &gnsspp::GsvSatellite::elev)
        .def_readonly("azim", &gnsspp::GsvSatellite::azim)
        .def_readonly("snr",  &gnsspp::GsvSatellite::snr);

    py::class_<gnsspp::NmeaGsv>(m, "NmeaGsv")
        .def_readonly("total_msgs", &gnsspp::NmeaGsv::total_msgs)
        .def_readonly("msg_num",    &gnsspp::NmeaGsv::msg_num)
        .def_readonly("total_svs",  &gnsspp::NmeaGsv::total_svs)
        .def_readonly("signal_id",  &gnsspp::NmeaGsv::signal_id)
        .def_readonly("satellites", &gnsspp::NmeaGsv::satellites);

    m.def("decode_gsv", &gnsspp::decode_gsv, py::arg("sentence"));

    py::class_<gnsspp::NmeaVtg>(m, "NmeaVtg")
        .def_readonly("course_true_deg", &gnsspp::NmeaVtg::course_true_deg)
        .def_readonly("course_mag_deg",  &gnsspp::NmeaVtg::course_mag_deg)
        .def_readonly("speed_knots",     &gnsspp::NmeaVtg::speed_knots)
        .def_readonly("speed_kmh",       &gnsspp::NmeaVtg::speed_kmh);

    m.def("decode_vtg", &gnsspp::decode_vtg, py::arg("sentence"));
}
