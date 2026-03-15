cmake_minimum_required(VERSION 3.16)
project(gnsspp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Library target ---------------------------------------------------------

set(GNSSPP_SOURCES
    src/frame_reader.cpp
    src/ubx/ubx_parser.cpp
    src/ubx/nav_parser.cpp
    src/ubx/nav_pvt.cpp
    src/ubx/nav_sat.cpp
    src/ubx/nav_svin.cpp
    src/ubx/nav_relposned.cpp
    src/ubx/nav_hpposllh.cpp
    src/ubx/nav_status.cpp
    src/ubx/rxm_parser.cpp
    src/decode.cpp
    src/ubx/rxm_rtcm.cpp
    src/ubx/cfg_valset.cpp
    src/ubx/cfg_rst.cpp
    src/ubx/ack.cpp
    src/nmea/nmea_parser.cpp
    src/nmea/gga.cpp
    src/nmea/rmc.cpp
    src/nmea/gsa.cpp
    src/nmea/gsv.cpp
    src/nmea/vtg.cpp
    src/rtcm3/rtcm3_parser.cpp
    src/rtcm3/bit_reader.cpp
    src/rtcm3/msg1005.cpp
    src/rtcm3/msg1006.cpp
    src/rtcm3/msg1230.cpp
    src/rtcm3/msm4.cpp
)
if(UNIX)
    list(APPEND GNSSPP_SOURCES src/posix_serial_port.cpp)
endif()

add_library(gnsspp ${GNSSPP_SOURCES})
add_library(gnsspp::gnsspp ALIAS gnsspp)

target_include_directories(gnsspp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(gnsspp PRIVATE -Wall -Wextra)
set_target_properties(gnsspp PROPERTIES
    VERSION ${PROJECT_VERSION}
    POSITION_INDEPENDENT_CODE ON
)

# ---- Install ----------------------------------------------------------------

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS gnsspp
    EXPORT gnssppTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT gnssppTargets
    FILE gnssppTargets.cmake
    NAMESPACE gnsspp::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gnsspp
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/gnssppConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gnssppConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/gnssppConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gnsspp
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/gnssppConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/gnssppConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gnsspp
)

# ---- Python bindings (scikit-build-core sets SKBUILD=1) ---------------------

set(GNSSPP_BINDING_SOURCES
    bindings/bindings.cpp
    bindings/bind_core.cpp
    bindings/bind_ubx.cpp
    bindings/bind_nmea.cpp
    bindings/bind_rtcm3.cpp
    bindings/bind_cfg.cpp
)

if(SKBUILD)
    if(NOT UNIX)
        message(FATAL_ERROR "gnsspp-py: Windows builds are not yet supported")
    endif()
    find_package(pybind11 REQUIRED)
    pybind11_add_module(_gnsspp ${GNSSPP_BINDING_SOURCES})
    target_link_libraries(_gnsspp PRIVATE gnsspp)
    target_compile_options(_gnsspp PRIVATE -Wall -Wextra)
    # install into gnsspp/ package dir so "import gnsspp" works via __init__.py
    install(TARGETS _gnsspp DESTINATION gnsspp)
endif()

# ---- Tests and examples (only when this is the top-level project) -----------

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    option(GNSSPP_BUILD_TESTS    "Build tests"    OFF)
    option(GNSSPP_BUILD_EXAMPLES "Build examples" ON)
    option(GNSSPP_BUILD_PYTHON   "Build Python bindings" OFF)

    if(GNSSPP_BUILD_PYTHON AND UNIX)
        find_package(pybind11 REQUIRED)
        pybind11_add_module(_gnsspp ${GNSSPP_BINDING_SOURCES})
        target_link_libraries(_gnsspp PRIVATE gnsspp)
        target_compile_options(_gnsspp PRIVATE -Wall -Wextra)
    endif()

    if(GNSSPP_BUILD_TESTS)
        include(FetchContent)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        v1.14.0
        )
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)

        file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
        add_executable(gnsspp_test ${TEST_SOURCES})
        target_link_libraries(gnsspp_test PRIVATE gnsspp GTest::gtest_main)
        target_compile_options(gnsspp_test PRIVATE -Wall -Wextra)
        target_include_directories(gnsspp_test PRIVATE test)

        include(GoogleTest)
        gtest_discover_tests(gnsspp_test)
    endif()

    if(GNSSPP_BUILD_EXAMPLES)
        add_executable(simple_reader examples/simple_reader.cpp)
        target_link_libraries(simple_reader PRIVATE gnsspp)
        target_compile_options(simple_reader PRIVATE -Wall -Wextra)

    endif()

endif()
