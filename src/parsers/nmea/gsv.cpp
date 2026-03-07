#include <stdexcept>

#include "gnsspp/error.hpp"
#include "gnsspp/parsers/nmea/gsv.hpp"
#include "nmea_util.hpp"


namespace gnsspp {

// $GPGSV,3,1,11,01,67,275,41,02,57,327,39,03,40,259,36,08,48,092,39,1*67
NmeaGsv decode_gsv(const std::string& sentence)
{
    auto f = nmea_util::split_fields(sentence);
    if (f.size() < 4) {
        throw ParseError("NMEA GSV: too few fields");
    }

    NmeaGsv out{};
    out.total_msgs = nmea_util::field_u8(f, 1);
    out.msg_num    = nmea_util::field_u8(f, 2);
    out.total_svs  = nmea_util::field_u8(f, 3);

    // SV records start at field[4], groups of 4: prn, elev, azim, snr
    // The last field may be signal_id (NMEA 4.11) if (fields - 4) % 4 == 1
    size_t data_fields = f.size() - 4;
    size_t sv_count    = data_fields / 4;
    out.signal_id = (data_fields % 4 == 1)
                    ? nmea_util::field_u8(f, f.size() - 1) : 0;

    out.satellites.reserve(sv_count);
    for (size_t i = 0; i < sv_count; ++i) {
        size_t base = 4 + i * 4;
        GsvSatellite sv{};
        sv.prn  = nmea_util::field_u8(f, base);
        sv.elev = (base+1 < f.size() && !f[base+1].empty())
                  ? static_cast<int8_t>(std::stoi(f[base+1])) : 0;
        sv.azim = (base+2 < f.size() && !f[base+2].empty())
                  ? static_cast<uint16_t>(std::stoi(f[base+2])) : 0;
        sv.snr  = nmea_util::field_u8(f, base+3);
        out.satellites.push_back(sv);
    }

    return out;
}


} // namespace gnsspp
