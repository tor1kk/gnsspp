# gnsspp

A lightweight C++17 library for parsing GNSS protocol frames — UBX, NMEA, and RTCM3.

## Supported messages

| Protocol | Messages |
|----------|----------|
| UBX      | NAV-PVT, NAV-SAT, NAV-SVIN, NAV-RELPOSNED, RXM-RTCM |
| NMEA     | GGA, RMC, GSA, GSV, VTG |
| RTCM3    | 1005, 1006, 1230, MSM4 (1074/1084/1094/1124) |

## Requirements

- C++17 compiler
- CMake 3.16+

## Usage

### FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(
    gnsspp
    GIT_REPOSITORY https://github.com/tor1kk/gnsspp.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(gnsspp)

target_link_libraries(my_app PRIVATE gnsspp::gnsspp)
```

### Quick example

```cpp
#include "gnsspp/gnsspp.hpp"

gnsspp::SerialPort port("/dev/ttyACM0", 38400);
port.open();

gnsspp::FrameReader reader(port);
reader.add_parser(std::make_unique<gnsspp::UBXParser>());
reader.add_parser(std::make_unique<gnsspp::NMEAParser>());
reader.add_parser(std::make_unique<gnsspp::RTCM3Parser>());

while (true) {
    auto frame = reader.read_frame();
    if (!frame) continue;

    if (frame->protocol == "UBX" && frame->type == "NAV-PVT") {
        auto msg = gnsspp::decode_nav_pvt(frame->payload());
        // msg.lat, msg.lon, msg.fix_type, msg.carr_soln ...
    }
}
```

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

To run tests:

```bash
cmake --build build --target gnsspp_test
ctest --test-dir build
```

## License

MIT
