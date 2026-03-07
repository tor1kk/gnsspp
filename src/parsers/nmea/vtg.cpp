#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/parsers/nmea/vtg.hpp"
#include "nmea_util.hpp"


namespace gnsspp {

// $GNVTG,0.0,T,,M,0.0,N,0.0,K,A*3D
NmeaVtg decode_vtg(const std::string& sentence)
{
    auto f = nmea_util::split_fields(sentence);
    if (f.size() < 9) {
        throw ParseError("NMEA VTG: too few fields");
    }

    NmeaVtg out{};
    out.course_true_deg = nmea_util::field_float(f, 1); // field[2] = 'T'
    out.course_mag_deg  = nmea_util::field_float(f, 3); // field[4] = 'M'
    out.speed_knots     = nmea_util::field_float(f, 5); // field[6] = 'N'
    out.speed_kmh       = nmea_util::field_float(f, 7); // field[8] = 'K'

    return out;
}


} // namespace gnsspp
