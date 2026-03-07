#include <stdexcept>
#include <string>

#include "gnsspp/error.hpp"
#include "gnsspp/parsers/nmea/gga.hpp"
#include "nmea_util.hpp"


namespace gnsspp {

NmeaGga decode_gga(const std::string& sentence)
{
    auto fields = nmea_util::split_fields(sentence);

    // Minimum: sentence_id + 9 data fields (up to altitude)
    if (fields.size() < 10) {
        throw ParseError("NMEA GGA: too few fields");
    }

    NmeaGga out{};

    out.utc = fields[1];

    // Latitude: DDMM.MMMM + N/S
    if (!fields[2].empty()) {
        out.lat = nmea_util::coord_to_deg(fields[2]);
        if (fields[3] == "S") out.lat = -out.lat;
    }

    // Longitude: DDDMM.MMMM + E/W
    if (!fields[4].empty()) {
        out.lon = nmea_util::coord_to_deg(fields[4]);
        if (fields[5] == "W") out.lon = -out.lon;
    }

    out.fix_quality = fields[6].empty() ? 0 : static_cast<uint8_t>(std::stoi(fields[6]));
    out.num_sv      = fields[7].empty() ? 0 : static_cast<uint8_t>(std::stoi(fields[7]));
    out.hdop        = fields[8].empty() ? 0.0f : std::stof(fields[8]);
    out.alt_m       = fields[9].empty() ? 0.0f : std::stof(fields[9]);
    // field[10] = altitude units (always 'M'), skip
    out.geoid_sep_m = (fields.size() > 11 && !fields[11].empty())
                      ? std::stof(fields[11]) : 0.0f;

    return out;
}


} // namespace gnsspp
