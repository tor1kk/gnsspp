#pragma once

#include <string>
#include <vector>


namespace gnsspp {
namespace nmea_util {

/// Split sentence fields on ',' and stop at '*' (start of checksum).
inline std::vector<std::string> split_fields(const std::string& sentence)
{
    std::vector<std::string> fields;
    std::string cur;
    for (char c : sentence) {
        if (c == ',') {
            fields.push_back(cur);
            cur.clear();
        } else if (c == '*') {
            fields.push_back(cur);
            break;
        } else {
            cur += c;
        }
    }
    return fields;
}

/// Convert NMEA coordinate string (DDMM.MMMM or DDDMM.MMMM) to decimal degrees.
inline double coord_to_deg(const std::string& val)
{
    if (val.empty()) return 0.0;
    double raw = std::stod(val);
    int    d   = static_cast<int>(raw / 100.0);
    double m   = raw - d * 100.0;
    return d + m / 60.0;
}

/// Parse a float field; return 0 if empty.
inline float field_float(const std::vector<std::string>& f, size_t i)
{
    return (i < f.size() && !f[i].empty()) ? std::stof(f[i]) : 0.0f;
}

/// Parse a uint8 field; return 0 if empty.
inline uint8_t field_u8(const std::vector<std::string>& f, size_t i)
{
    return (i < f.size() && !f[i].empty())
        ? static_cast<uint8_t>(std::stoi(f[i])) : 0;
}

} // namespace nmea_util
} // namespace gnsspp
