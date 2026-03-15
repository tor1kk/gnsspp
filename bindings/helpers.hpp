#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <pybind11/pybind11.h>

namespace py = pybind11;

inline std::vector<uint8_t> bytes_to_vec(py::bytes b)
{
    std::string s = b;
    return {s.begin(), s.end()};
}

inline py::bytes vec_to_bytes(const std::vector<uint8_t>& v)
{
    return py::bytes(reinterpret_cast<const char*>(v.data()), v.size());
}
