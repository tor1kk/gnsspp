#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "gnsspp/ubx/cfg_valset.hpp"
#include "gnsspp/ubx/cfg_rst.hpp"

#include "helpers.hpp"

namespace py = pybind11;

// ---------------------------------------------------------------------------
// UbxFramer — simple stateful streaming UBX frame extractor.
// Feed raw bytes from pyserial; pop complete frames with next_frame().
// ---------------------------------------------------------------------------

static bool ubx_checksum_ok(const std::vector<uint8_t>& buf,
                             size_t start, size_t payload_len)
{
    uint8_t ck_a = 0, ck_b = 0;
    size_t  end  = start + 4 + payload_len;
    for (size_t i = start; i < end; i++) {
        ck_a += buf[i];
        ck_b += ck_a;
    }
    return ck_a == buf[end] && ck_b == buf[end + 1];
}

class UbxFramer {
public:
    void feed(py::bytes data) {
        std::string s = data;
        buf_.insert(buf_.end(), s.begin(), s.end());
    }

    // Returns (cls: int, id: int, payload: bytes) or None.
    py::object next_frame() {
        while (buf_.size() >= 8) {
            if (buf_[0] != 0xB5 || buf_[1] != 0x62) {
                buf_.erase(buf_.begin());
                continue;
            }
            uint16_t payload_len = static_cast<uint16_t>(buf_[4]) |
                                   (static_cast<uint16_t>(buf_[5]) << 8);
            size_t total = 6u + payload_len + 2u;
            if (buf_.size() < total)
                return py::none();

            if (!ubx_checksum_ok(buf_, 2, payload_len)) {
                buf_.erase(buf_.begin());
                continue;
            }

            uint8_t   cls    = buf_[2];
            uint8_t   msg_id = buf_[3];
            py::bytes payload(reinterpret_cast<const char*>(buf_.data() + 6), payload_len);
            buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
            return py::make_tuple(cls, msg_id, payload);
        }
        return py::none();
    }

private:
    std::vector<uint8_t> buf_;
};

// ---------------------------------------------------------------------------

void bind_cfg(py::module_& m)
{
    py::class_<UbxFramer>(m, "UbxFramer",
        "Stateful streaming UBX frame extractor.\n\n"
        "Feed raw bytes with feed(); retrieve complete frames with next_frame()\n"
        "which returns (cls, id, payload: bytes) or None.")
        .def(py::init<>())
        .def("feed",       &UbxFramer::feed,       "Append raw bytes to internal buffer.")
        .def("next_frame", &UbxFramer::next_frame,  "Pop next UBX frame or return None.");

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
