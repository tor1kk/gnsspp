#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/parsers/nmea/rmc.hpp"
#include "nmea_util.hpp"


namespace gnsspp {

// $GNRMC,123519.00,A,4807.0380,N,01131.0000,E,0.0,,010324,,,A*68
NmeaRmc decode_rmc(const std::string& sentence)
{
    auto f = nmea_util::split_fields(sentence);
    if (f.size() < 10) {
        throw ParseError("NMEA RMC: too few fields");
    }

    NmeaRmc out{};
    out.utc          = f[1];
    out.active       = (!f[2].empty() && f[2][0] == 'A');
    out.lat          = nmea_util::coord_to_deg(f[3]);
    if (!f[4].empty() && f[4][0] == 'S') out.lat = -out.lat;
    out.lon          = nmea_util::coord_to_deg(f[5]);
    if (!f[6].empty() && f[6][0] == 'W') out.lon = -out.lon;
    out.speed_knots  = nmea_util::field_float(f, 7);
    out.course_deg   = nmea_util::field_float(f, 8);
    out.date         = f[9];

    return out;
}


} // namespace gnsspp
