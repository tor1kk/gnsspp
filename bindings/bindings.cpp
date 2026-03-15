#include <pybind11/pybind11.h>

#include "bind_core.hpp"
#include "bind_ubx.hpp"
#include "bind_nmea.hpp"
#include "bind_rtcm3.hpp"
#include "bind_cfg.hpp"

PYBIND11_MODULE(_gnsspp, m)
{
    m.doc() = "gnsspp — GNSS protocol library Python bindings";
    bind_core(m);
    bind_ubx(m);
    bind_nmea(m);
    bind_rtcm3(m);
    bind_cfg(m);
}
