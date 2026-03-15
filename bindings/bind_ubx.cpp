#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "gnsspp/ubx/nav_pvt.hpp"
#include "gnsspp/ubx/nav_hpposllh.hpp"
#include "gnsspp/ubx/nav_status.hpp"
#include "gnsspp/ubx/nav_sat.hpp"
#include "gnsspp/ubx/nav_svin.hpp"
#include "gnsspp/ubx/nav_relposned.hpp"
#include "gnsspp/ubx/rxm_rtcm.hpp"
#include "gnsspp/ubx/ack.hpp"

#include "helpers.hpp"

namespace py = pybind11;

void bind_ubx(py::module_& m)
{
    py::class_<gnsspp::NavPvt>(m, "NavPvt")
        .def_readonly("itow",        &gnsspp::NavPvt::itow)
        .def_readonly("year",        &gnsspp::NavPvt::year)
        .def_readonly("month",       &gnsspp::NavPvt::month)
        .def_readonly("day",         &gnsspp::NavPvt::day)
        .def_readonly("hour",        &gnsspp::NavPvt::hour)
        .def_readonly("min",         &gnsspp::NavPvt::min)
        .def_readonly("second",      &gnsspp::NavPvt::second)
        .def_readonly("fix_type",    &gnsspp::NavPvt::fix_type)
        .def_readonly("num_sv",      &gnsspp::NavPvt::num_sv)
        .def_readonly("gnss_fix_ok", &gnsspp::NavPvt::gnss_fix_ok)
        .def_readonly("carr_soln",   &gnsspp::NavPvt::carr_soln)
        .def_readonly("lon",         &gnsspp::NavPvt::lon)
        .def_readonly("lat",         &gnsspp::NavPvt::lat)
        .def_readonly("height",      &gnsspp::NavPvt::height)
        .def_readonly("h_msl",       &gnsspp::NavPvt::h_msl)
        .def_readonly("h_acc",       &gnsspp::NavPvt::h_acc)
        .def_readonly("v_acc",       &gnsspp::NavPvt::v_acc)
        .def_readonly("vel_n",       &gnsspp::NavPvt::vel_n)
        .def_readonly("vel_e",       &gnsspp::NavPvt::vel_e)
        .def_readonly("vel_d",       &gnsspp::NavPvt::vel_d)
        .def_readonly("g_speed",     &gnsspp::NavPvt::g_speed)
        .def_readonly("head_mot",    &gnsspp::NavPvt::head_mot)
        .def_readonly("s_acc",       &gnsspp::NavPvt::s_acc)
        .def_readonly("p_dop",       &gnsspp::NavPvt::p_dop);

    m.def("decode_nav_pvt", [](py::bytes b) {
        return gnsspp::decode_nav_pvt(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::SvInfo>(m, "SvInfo")
        .def_readonly("gnss_id",        &gnsspp::SvInfo::gnss_id)
        .def_readonly("sv_id",          &gnsspp::SvInfo::sv_id)
        .def_readonly("cno",            &gnsspp::SvInfo::cno)
        .def_readonly("elev",           &gnsspp::SvInfo::elev)
        .def_readonly("azim",           &gnsspp::SvInfo::azim)
        .def_readonly("pr_res",         &gnsspp::SvInfo::pr_res)
        .def_readonly("quality_ind",    &gnsspp::SvInfo::quality_ind)
        .def_readonly("sv_used",        &gnsspp::SvInfo::sv_used)
        .def_readonly("health",         &gnsspp::SvInfo::health)
        .def_readonly("diff_corr",      &gnsspp::SvInfo::diff_corr)
        .def_readonly("smoothed",       &gnsspp::SvInfo::smoothed)
        .def_readonly("orbit_source",   &gnsspp::SvInfo::orbit_source)
        .def_readonly("eph_avail",         &gnsspp::SvInfo::eph_avail)
        .def_readonly("alm_avail",         &gnsspp::SvInfo::alm_avail)
        .def_readonly("ano_avail",         &gnsspp::SvInfo::ano_avail)
        .def_readonly("aop_avail",         &gnsspp::SvInfo::aop_avail)
        .def_readonly("sbas_corr_used",    &gnsspp::SvInfo::sbas_corr_used)
        .def_readonly("rtcm_corr_used",    &gnsspp::SvInfo::rtcm_corr_used)
        .def_readonly("slas_corr_used",    &gnsspp::SvInfo::slas_corr_used)
        .def_readonly("spartn_corr_used",  &gnsspp::SvInfo::spartn_corr_used)
        .def_readonly("pr_corr_used",      &gnsspp::SvInfo::pr_corr_used)
        .def_readonly("cr_corr_used",      &gnsspp::SvInfo::cr_corr_used)
        .def_readonly("do_corr_used",      &gnsspp::SvInfo::do_corr_used)
        .def_readonly("clas_corr_used",    &gnsspp::SvInfo::clas_corr_used);

    py::class_<gnsspp::NavSat>(m, "NavSat")
        .def_readonly("itow",    &gnsspp::NavSat::itow)
        .def_readonly("num_svs", &gnsspp::NavSat::num_svs)
        .def_readonly("svs",     &gnsspp::NavSat::svs);

    m.def("decode_nav_sat", [](py::bytes b) {
        return gnsspp::decode_nav_sat(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::NavRelPosNed>(m, "NavRelPosNed")
        .def_readonly("itow",                  &gnsspp::NavRelPosNed::itow)
        .def_readonly("ref_station_id",        &gnsspp::NavRelPosNed::ref_station_id)
        .def_readonly("rel_pos_n_m",           &gnsspp::NavRelPosNed::rel_pos_n_m)
        .def_readonly("rel_pos_e_m",           &gnsspp::NavRelPosNed::rel_pos_e_m)
        .def_readonly("rel_pos_d_m",           &gnsspp::NavRelPosNed::rel_pos_d_m)
        .def_readonly("rel_pos_length_m",      &gnsspp::NavRelPosNed::rel_pos_length_m)
        .def_readonly("rel_pos_heading_deg",   &gnsspp::NavRelPosNed::rel_pos_heading_deg)
        .def_readonly("acc_n_m",               &gnsspp::NavRelPosNed::acc_n_m)
        .def_readonly("acc_e_m",               &gnsspp::NavRelPosNed::acc_e_m)
        .def_readonly("acc_d_m",               &gnsspp::NavRelPosNed::acc_d_m)
        .def_readonly("gnss_fix_ok",           &gnsspp::NavRelPosNed::gnss_fix_ok)
        .def_readonly("diff_soln",             &gnsspp::NavRelPosNed::diff_soln)
        .def_readonly("rel_pos_valid",         &gnsspp::NavRelPosNed::rel_pos_valid)
        .def_readonly("carr_soln",             &gnsspp::NavRelPosNed::carr_soln)
        .def_readonly("rel_pos_heading_valid", &gnsspp::NavRelPosNed::rel_pos_heading_valid);

    m.def("decode_nav_relposned", [](py::bytes b) {
        return gnsspp::decode_nav_relposned(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::NavSvIn>(m, "NavSvIn")
        .def_readonly("itow",       &gnsspp::NavSvIn::itow)
        .def_readonly("dur",        &gnsspp::NavSvIn::dur)
        .def_readonly("mean_x_m",   &gnsspp::NavSvIn::mean_x_m)
        .def_readonly("mean_y_m",   &gnsspp::NavSvIn::mean_y_m)
        .def_readonly("mean_z_m",   &gnsspp::NavSvIn::mean_z_m)
        .def_readonly("mean_acc_m", &gnsspp::NavSvIn::mean_acc_m)
        .def_readonly("obs",        &gnsspp::NavSvIn::obs)
        .def_readonly("valid",      &gnsspp::NavSvIn::valid)
        .def_readonly("active",     &gnsspp::NavSvIn::active);

    m.def("decode_nav_svin", [](py::bytes b) {
        return gnsspp::decode_nav_svin(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::AckAck>(m, "AckAck")
        .def_readonly("cls_id", &gnsspp::AckAck::cls_id)
        .def_readonly("msg_id", &gnsspp::AckAck::msg_id);

    py::class_<gnsspp::AckNak>(m, "AckNak")
        .def_readonly("cls_id", &gnsspp::AckNak::cls_id)
        .def_readonly("msg_id", &gnsspp::AckNak::msg_id);

    m.def("decode_ack_ack", [](py::bytes b) {
        return gnsspp::decode_ack_ack(bytes_to_vec(b));
    }, py::arg("payload"));

    m.def("decode_ack_nak", [](py::bytes b) {
        return gnsspp::decode_ack_nak(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::RxmRtcm>(m, "RxmRtcm")
        .def_readonly("msg_type",    &gnsspp::RxmRtcm::msg_type)
        .def_readonly("ref_station", &gnsspp::RxmRtcm::ref_station)
        .def_readonly("crc_failed",  &gnsspp::RxmRtcm::crc_failed);

    m.def("decode_rxm_rtcm", [](py::bytes b) {
        return gnsspp::decode_rxm_rtcm(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::NavHpPosLlh>(m, "NavHpPosLlh")
        .def_readonly("itow",        &gnsspp::NavHpPosLlh::itow)
        .def_readonly("invalid_llh", &gnsspp::NavHpPosLlh::invalid_llh)
        .def_readonly("lon",         &gnsspp::NavHpPosLlh::lon)
        .def_readonly("lat",         &gnsspp::NavHpPosLlh::lat)
        .def_readonly("height",      &gnsspp::NavHpPosLlh::height)
        .def_readonly("h_msl",       &gnsspp::NavHpPosLlh::h_msl)
        .def_readonly("h_acc",       &gnsspp::NavHpPosLlh::h_acc)
        .def_readonly("v_acc",       &gnsspp::NavHpPosLlh::v_acc);

    m.def("decode_nav_hpposllh", [](py::bytes b) {
        return gnsspp::decode_nav_hpposllh(bytes_to_vec(b));
    }, py::arg("payload"));

    py::class_<gnsspp::NavStatus>(m, "NavStatus")
        .def_readonly("itow",             &gnsspp::NavStatus::itow)
        .def_readonly("gps_fix",          &gnsspp::NavStatus::gps_fix)
        .def_readonly("gps_fix_ok",       &gnsspp::NavStatus::gps_fix_ok)
        .def_readonly("diff_soln",        &gnsspp::NavStatus::diff_soln)
        .def_readonly("carr_soln_valid",  &gnsspp::NavStatus::carr_soln_valid)
        .def_readonly("carr_soln",        &gnsspp::NavStatus::carr_soln)
        .def_readonly("ttff",             &gnsspp::NavStatus::ttff)
        .def_readonly("msss",             &gnsspp::NavStatus::msss);

    m.def("decode_nav_status", [](py::bytes b) {
        return gnsspp::decode_nav_status(bytes_to_vec(b));
    }, py::arg("payload"));
}
