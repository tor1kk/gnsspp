#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "gnsspp/ubx/cfg_valset.hpp"
#include "gnsspp/ubx/cfg_valget.hpp"
#include "gnsspp/ubx/cfg_rst.hpp"
#include "gnsspp/ubx/ubx_poll.hpp"

#include "helpers.hpp"

namespace py = pybind11;

void bind_cfg(py::module_& m)
{
    // ---- CFG-VALSET --------------------------------------------------------

    m.def("build_cfg_valset",
        [](const std::vector<std::pair<uint32_t, uint64_t>>& items, uint8_t layers) {
            std::vector<gnsspp::CfgVal> vals;
            vals.reserve(items.size());
            for (auto& [k, v] : items)
                vals.push_back({k, v});
            return vec_to_bytes(gnsspp::build_cfg_valset(vals, layers));
        },
        py::arg("items"),
        py::arg("layers") = static_cast<uint8_t>(gnsspp::CFG_LAYER_RAM),
        "Build a UBX-CFG-VALSET frame.\n\n"
        "items  — list of (key: int, value: int) pairs\n"
        "layers — bitmask: CFG_LAYER_RAM | CFG_LAYER_BBR | CFG_LAYER_FLASH");

    // ---- CFG-VALGET --------------------------------------------------------

    m.def("build_cfg_valget",
        [](const std::vector<uint32_t>& keys, uint8_t layer, uint16_t position) {
            return vec_to_bytes(gnsspp::build_cfg_valget(keys, layer, position));
        },
        py::arg("keys"),
        py::arg("layer")    = static_cast<uint8_t>(gnsspp::CFG_GET_LAYER_RAM),
        py::arg("position") = static_cast<uint16_t>(0),
        "Build a UBX-CFG-VALGET request frame.\n\n"
        "keys     — list of configuration key IDs to query\n"
        "layer    — CFG_GET_LAYER_RAM / BBR / FLASH / DEFAULT\n"
        "position — pagination offset (0 for first page)");

    m.def("parse_cfg_valget",
        [](py::bytes payload) {
            std::string s = payload;
            std::vector<uint8_t> buf(s.begin(), s.end());
            std::vector<std::pair<uint32_t, uint64_t>> result;
            for (const auto& item : gnsspp::parse_cfg_valget(buf))
                result.emplace_back(item.key, item.value);
            return result;
        },
        py::arg("payload"),
        "Parse the payload of a UBX-CFG-VALGET response.\n\n"
        "Returns a list of (key: int, value: int) pairs.");

    m.attr("CFG_GET_LAYER_RAM")     = static_cast<int>(gnsspp::CFG_GET_LAYER_RAM);
    m.attr("CFG_GET_LAYER_BBR")     = static_cast<int>(gnsspp::CFG_GET_LAYER_BBR);
    m.attr("CFG_GET_LAYER_FLASH")   = static_cast<int>(gnsspp::CFG_GET_LAYER_FLASH);
    m.attr("CFG_GET_LAYER_DEFAULT") = static_cast<int>(gnsspp::CFG_GET_LAYER_DEFAULT);

    m.def("build_ubx_poll",
        [](uint8_t cls, uint8_t msg_id) {
            return vec_to_bytes(gnsspp::build_ubx_poll(cls, msg_id));
        },
        py::arg("cls"),
        py::arg("msg_id"),
        "Build a UBX poll request frame (empty payload) for the given class and message ID.");

    m.def("build_cfg_rst",
        [](uint16_t nav_bbr_mask, uint8_t reset_mode) {
            return vec_to_bytes(gnsspp::build_cfg_rst(nav_bbr_mask, reset_mode));
        },
        py::arg("nav_bbr_mask"),
        py::arg("reset_mode"),
        "Build a UBX-CFG-RST frame.");

    // ---- Layer constants ---------------------------------------------------

    m.attr("CFG_LAYER_RAM")   = static_cast<int>(gnsspp::CFG_LAYER_RAM);
    m.attr("CFG_LAYER_BBR")   = static_cast<int>(gnsspp::CFG_LAYER_BBR);
    m.attr("CFG_LAYER_FLASH") = static_cast<int>(gnsspp::CFG_LAYER_FLASH);
    m.attr("CFG_LAYER_ALL")   = static_cast<int>(gnsspp::CFG_LAYER_ALL);

    // ---- Reset constants ---------------------------------------------------

    m.attr("CFG_RST_HOT")      = static_cast<int>(gnsspp::CFG_RST_HOT);
    m.attr("CFG_RST_WARM")     = static_cast<int>(gnsspp::CFG_RST_WARM);
    m.attr("CFG_RST_COLD")     = static_cast<int>(gnsspp::CFG_RST_COLD);
    m.attr("CFG_RST_MODE_HW")  = static_cast<int>(gnsspp::CFG_RST_MODE_HW);
    m.attr("CFG_RST_MODE_SW")  = static_cast<int>(gnsspp::CFG_RST_MODE_SW);
    m.attr("CFG_RST_MODE_GNSS")= static_cast<int>(gnsspp::CFG_RST_MODE_GNSS);
}
