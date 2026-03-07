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
        auto msg = gnsspp::decode_nav_pvt(payload);
        std::cout << "[NAV-PVT] fix=" << static_cast<int>(msg.fix_type)
                  << " lat=" << msg.lat << " lon=" << msg.lon
                  << " alt=" << msg.height / 1000.0 << "m"
                  << " numSV=" << static_cast<int>(msg.num_sv)
                  << " carrSoln=" << static_cast<int>(msg.carr_soln)
                  << "\n";
    } else if (frame.type == "NAV-SAT") {
        auto msg = gnsspp::decode_nav_sat(payload);
        std::cout << "[NAV-SAT] " << msg.num_svs << " satellites tracked\n";
    } else if (frame.type == "NAV-SVIN") {
        auto msg = gnsspp::decode_nav_svin(payload);
        std::cout << "[NAV-SVIN] dur=" << msg.dur << "s"
                  << " valid=" << msg.valid
                  << " acc=" << msg.mean_acc_m * 1000.0 << "mm\n";
    } else if (!frame.type.empty()) {
        std::cout << "[UBX] unhandled type=" << frame.type << "\n";
    }
    // frame.type.empty() → unknown id, silently skip
}

static void handle_nmea(const gnsspp::Frame& frame)
{
    std::string sentence(frame.raw.begin(), frame.raw.end());

    if (frame.type == "GGA") {
        auto msg = gnsspp::decode_gga(sentence);
        std::cout << "[GGA] fix=" << static_cast<int>(msg.fix_quality)
                  << " lat=" << msg.lat << " lon=" << msg.lon
                  << " alt=" << msg.alt_m << "m"
                  << " numSV=" << static_cast<int>(msg.num_sv)
                  << "\n";
    } else if (frame.type == "RMC") {
        auto msg = gnsspp::decode_rmc(sentence);
        std::cout << "[RMC] active=" << msg.active
                  << " lat=" << msg.lat << " lon=" << msg.lon
                  << " speed=" << msg.speed_knots << "kt\n";
    }
    // other NMEA types: skip
}

static void handle_rtcm3(const gnsspp::Frame& frame)
{
    auto payload = frame.payload();

    if (frame.type == "1005") {
        auto msg = gnsspp::decode_msg1005(payload);
        std::cout << "[RTCM 1005] station=" << msg.station_id
                  << " X=" << msg.ecef_x << " Y=" << msg.ecef_y
                  << " Z=" << msg.ecef_z << " m\n";
    } else if (frame.type == "1074" || frame.type == "1084"
            || frame.type == "1094" || frame.type == "1124") {
        auto msg = gnsspp::decode_msm4(payload);
        std::cout << "[RTCM " << frame.type << "] station=" << msg.station_id
                  << " sats=" << msg.satellites.size()
                  << " signals=" << msg.signals.size() << "\n";
    }
}


int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <device> <baudrate>\n"
                  << "  e.g. " << argv[0] << " /dev/ttyACM0 115200\n";
        return EXIT_FAILURE;
    }

    const std::string device   = argv[1];
    const int         baudrate = std::stoi(argv[2]);

    gnsspp::SerialPort port(device, baudrate);

    try {
        port.open();
    } catch (const gnsspp::IoError& e) {
        std::cerr << "Failed to open " << device << ": " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    gnsspp::FrameReader reader(port);
    reader.add_parser(std::make_unique<gnsspp::UBXParser>());
    reader.add_parser(std::make_unique<gnsspp::NMEAParser>());
    reader.add_parser(std::make_unique<gnsspp::RTCM3Parser>());

    std::cout << "Reading from " << device << " at " << baudrate << " baud...\n";

    while (true) {
        try {
            auto frame = reader.read_frame();
            if (!frame) continue;  // timeout with no data

            if      (frame->protocol == "UBX")   handle_ubx(*frame);
            else if (frame->protocol == "NMEA")  handle_nmea(*frame);
            else if (frame->protocol == "RTCM3") handle_rtcm3(*frame);

        } catch (const gnsspp::ParseError& e) {
            // Malformed frame — log and keep going
            std::cerr << "[WARN] parse error: " << e.what() << "\n";
        } catch (const gnsspp::IoError& e) {
            // Port problem — likely unrecoverable
            std::cerr << "[ERROR] I/O error: " << e.what() << "\n";
            break;
        }
    }

    port.close();
    return EXIT_SUCCESS;
}
