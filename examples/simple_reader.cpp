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


static void handle_ubx(const gnsspp::Frame& frame)
{
    auto payload = frame.payload();

    if (frame.type == "NAV-PVT") {
        auto m = gnsspp::decode_nav_pvt(payload);
        const char* carr[] = {"none", "float", "fixed"};
        std::cout << "[NAV-PVT]  fix=" << static_cast<int>(m.fix_type)
                  << " lat=" << m.lat << " lon=" << m.lon
                  << " alt=" << m.height / 1000.0 << "m"
                  << " numSV=" << static_cast<int>(m.num_sv)
                  << " RTK=" << carr[m.carr_soln] << "\n";

    } else if (frame.type == "NAV-HPPOSLLH") {
        auto m = gnsspp::decode_nav_hpposllh(payload);
        if (!m.invalid_llh)
            std::cout << "[HPPOSLLH] lat=" << m.lat << " lon=" << m.lon
                      << " hAcc=" << m.h_acc << "mm vAcc=" << m.v_acc << "mm\n";

    } else if (frame.type == "NAV-SVIN") {
        auto m = gnsspp::decode_nav_svin(payload);
        std::cout << "[NAV-SVIN] dur=" << m.dur << "s"
                  << " valid=" << m.valid
                  << " acc=" << m.mean_acc_m * 1000.0 << "mm\n";

    } else if (frame.type == "NAV-SAT") {
        auto m = gnsspp::decode_nav_sat(payload);
        std::cout << "[NAV-SAT]  " << static_cast<int>(m.num_svs)
                  << " satellites\n";
    }
}

static void handle_nmea(const gnsspp::Frame& frame)
{
    std::string sentence(frame.raw.begin(), frame.raw.end());

    if (frame.type == "GGA") {
        auto m = gnsspp::decode_gga(sentence);
        std::cout << "[GGA] fix=" << static_cast<int>(m.fix_quality)
                  << " lat=" << m.lat << " lon=" << m.lon
                  << " alt=" << m.alt_m << "m"
                  << " numSV=" << static_cast<int>(m.num_sv) << "\n";

    } else if (frame.type == "RMC") {
        auto m = gnsspp::decode_rmc(sentence);
        std::cout << "[RMC] active=" << m.active
                  << " lat=" << m.lat << " lon=" << m.lon
                  << " speed=" << m.speed_knots << "kt\n";
    }
}

static void handle_rtcm3(const gnsspp::Frame& frame)
{
    auto payload = frame.payload();

    if (frame.type == "1005") {
        auto m = gnsspp::decode_msg1005(payload);
        std::cout << "[RTCM 1005] station=" << m.station_id
                  << " X=" << m.ecef_x << " Y=" << m.ecef_y
                  << " Z=" << m.ecef_z << " m\n";

    } else if (frame.type == "1074" || frame.type == "1084"
            || frame.type == "1094" || frame.type == "1124") {
        auto m = gnsspp::decode_msm4(payload);
        std::cout << "[RTCM " << frame.type << "] station=" << m.station_id
                  << " sats=" << m.satellites.size()
                  << " signals=" << m.signals.size() << "\n";
    }
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

            if      (frame->protocol == "UBX")   handle_ubx(*frame);
            else if (frame->protocol == "NMEA")  handle_nmea(*frame);
            else if (frame->protocol == "RTCM3") handle_rtcm3(*frame);

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
