// simple_reader.cpp — minimal example: read UBX/NMEA/RTCM3 frames from a
// serial port and decode the most common message types.
//
// Build (after cmake):
//   cmake --build build --target simple_reader
//
// Run:
//   ./build/simple_reader /dev/ttyACM0 115200

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "gnsspp/gnsspp.hpp"


static void handle(const gnsspp::Message& msg)
{
    std::visit([](auto&& m) {
        using T = std::decay_t<decltype(m)>;

        if constexpr (std::is_same_v<T, gnsspp::NavPvt>) {
            const char* carr[] = {"none", "float", "fixed"};
            std::cout << "[NAV-PVT]  fix=" << static_cast<int>(m.fix_type)
                      << " lat=" << m.lat << " lon=" << m.lon
                      << " alt=" << m.height / 1000.0 << "m"
                      << " numSV=" << static_cast<int>(m.num_sv)
                      << " RTK=" << carr[m.carr_soln] << "\n";

        } else if constexpr (std::is_same_v<T, gnsspp::NavHpPosLlh>) {
            if (!m.invalid_llh)
                std::cout << "[HPPOSLLH] lat=" << m.lat << " lon=" << m.lon
                          << " hAcc=" << m.h_acc << "mm vAcc=" << m.v_acc << "mm\n";

        } else if constexpr (std::is_same_v<T, gnsspp::NavSvIn>) {
            std::cout << "[NAV-SVIN] dur=" << m.dur << "s"
                      << " valid=" << m.valid
                      << " acc=" << m.mean_acc_m * 1000.0 << "mm\n";

        } else if constexpr (std::is_same_v<T, gnsspp::NavSat>) {
            std::cout << "[NAV-SAT]  " << static_cast<int>(m.num_svs)
                      << " satellites\n";

        } else if constexpr (std::is_same_v<T, gnsspp::NmeaGga>) {
            std::cout << "[GGA] fix=" << static_cast<int>(m.fix_quality)
                      << " lat=" << m.lat << " lon=" << m.lon
                      << " alt=" << m.alt_m << "m"
                      << " numSV=" << static_cast<int>(m.num_sv) << "\n";

        } else if constexpr (std::is_same_v<T, gnsspp::Msg1005>) {
            std::cout << "[RTCM 1005] station=" << m.station_id
                      << " X=" << m.ecef_x << " Y=" << m.ecef_y
                      << " Z=" << m.ecef_z << " m\n";

        } else if constexpr (std::is_same_v<T, gnsspp::Msm4>) {
            std::cout << "[RTCM " << m.msg_type << "] station=" << m.station_id
                      << " sats=" << m.satellites.size()
                      << " signals=" << m.signals.size() << "\n";
        }
    }, msg);
}


int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <device> <baudrate>\n"
                  << "  e.g. " << argv[0] << " /dev/ttyACM0 115200\n";
        return EXIT_FAILURE;
    }

    gnsspp::PosixSerialPort port(argv[1], std::stoi(argv[2]));

    try {
        port.open();
    } catch (const gnsspp::IoError& e) {
        std::cerr << "Failed to open " << argv[1] << ": " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    gnsspp::FrameReader reader(port);
    reader.add_parser(std::make_unique<gnsspp::UBXParser>());
    reader.add_parser(std::make_unique<gnsspp::NMEAParser>());
    reader.add_parser(std::make_unique<gnsspp::RTCM3Parser>());

    std::cout << "Reading from " << argv[1] << " at " << argv[2] << " baud...\n";

    while (true) {
        try {
            auto frame = reader.read_frame();
            if (!frame) continue;

            if (auto msg = gnsspp::decode(*frame))
                handle(*msg);

        } catch (const gnsspp::ParseError& e) {
            std::cerr << "[WARN] " << e.what() << "\n";
        } catch (const gnsspp::IoError& e) {
            std::cerr << "[ERROR] " << e.what() << "\n";
            break;
        }
    }

    port.close();
    return EXIT_SUCCESS;
}
