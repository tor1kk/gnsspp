#include <cstring>
#include <stdexcept>
#include <string>

#include "gnsspp/error.hpp"
#include "gnsspp/parsers/nmea/gsa.hpp"
#include "nmea_util.hpp"


namespace gnsspp {

// $GNGSA,A,3,01,02,03,08,09,10,16,23,31,,,,1.23,0.65,1.04,1*0A
NmeaGsa decode_gsa(const std::string& sentence)
{
    auto f = nmea_util::split_fields(sentence);
    if (f.size() < 18) {
        throw ParseError("NMEA GSA: too few fields");
    }

    NmeaGsa out{};
    std::memset(out.sv_prns, 0, sizeof(out.sv_prns));

    out.auto_mode = (!f[1].empty() && f[1][0] == 'A');
    out.fix_type  = nmea_util::field_u8(f, 2);

    // Fields [3..14]: up to 12 SV PRNs
    out.sv_count = 0;
    for (size_t i = 3; i <= 14 && i < f.size(); ++i) {
        if (!f[i].empty()) {
            out.sv_prns[out.sv_count++] = static_cast<uint8_t>(std::stoi(f[i]));
        }
    }

    out.pdop      = nmea_util::field_float(f, 15);
    out.hdop      = nmea_util::field_float(f, 16);
    out.vdop      = nmea_util::field_float(f, 17);
    out.system_id = nmea_util::field_u8(f, 18);  // NMEA 4.11; absent in older receivers

    return out;
}


} // namespace gnsspp
