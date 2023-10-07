cmake_minimum_required(VERSION 3.5)
project(controller)

add_subdirectory(lib/UAV_common)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(SOURCES
    ${SOURCE_DIR}/communication/control_recv.cpp
    ${SOURCE_DIR}/communication/control_send.cpp
    ${SOURCE_DIR}/communication/control.cpp
    ${SOURCE_DIR}/communication/control.hpp
    ${SOURCE_DIR}/controller/controller.cpp
    ${SOURCE_DIR}/controller/controller.hpp
    ${SOURCE_DIR}/controller/controller_loop.cpp
    ${SOURCE_DIR}/controller/controller_loop.hpp
    ${SOURCE_DIR}/controller/controller_mode.hpp
    ${SOURCE_DIR}/controller/mixers.cpp
    ${SOURCE_DIR}/controller/mixers.hpp
    ${SOURCE_DIR}/controller/modes/controller_loop_FMANUAL.cpp
    ${SOURCE_DIR}/controller/modes/controller_loop_FMANUAL.hpp
    ${SOURCE_DIR}/controller/modes/controller_loop_NONE.cpp
    ${SOURCE_DIR}/controller/modes/controller_loop_NONE.hpp
    ${SOURCE_DIR}/controller/modes/controller_loop_QACRO.cpp
    ${SOURCE_DIR}/controller/modes/controller_loop_QACRO.hpp
    ${SOURCE_DIR}/controller/modes/controller_loop_QANGLE.cpp
    ${SOURCE_DIR}/controller/modes/controller_loop_QANGLE.hpp
    ${SOURCE_DIR}/controller/modes/controller_loop_QPOS.cpp
    ${SOURCE_DIR}/controller/modes/controller_loop_QPOS.hpp
    ${SOURCE_DIR}/controller/modes/controller_loop_RMANUAL.cpp
    ${SOURCE_DIR}/controller/modes/controller_loop_RMANUAL.hpp
    ${SOURCE_DIR}/defines.hpp
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/navigation/AHRS/AHRS_complementary.cpp
    ${SOURCE_DIR}/navigation/AHRS/AHRS_complementary.hpp
    ${SOURCE_DIR}/navigation/AHRS/AHRS_EKF.cpp
    ${SOURCE_DIR}/navigation/AHRS/AHRS_EKF.hpp
    ${SOURCE_DIR}/navigation/AHRS.cpp
    ${SOURCE_DIR}/navigation/AHRS.hpp
    ${SOURCE_DIR}/navigation/EKF.cpp
    ${SOURCE_DIR}/navigation/EKF.hpp
    ${SOURCE_DIR}/navigation/environment.cpp
    ${SOURCE_DIR}/navigation/environment.hpp
    ${SOURCE_DIR}/navigation/NS.cpp
    ${SOURCE_DIR}/navigation/NS.hpp
    ${SOURCE_DIR}/navigation/sensors.cpp
    ${SOURCE_DIR}/navigation/sensors.hpp
    ${SOURCE_DIR}/utils.hpp
)

include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR})
link_directories(${CMAKE_SOURCE_DIR}/lib/UAV_common)

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")


add_executable(controller ${SOURCES})
set_property(TARGET controller PROPERTY CXX_STANDARD 20)
target_compile_features(controller PUBLIC cxx_std_20)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(controller Eigen3::Eigen)
find_package(cppzmq)
target_link_libraries(controller cppzmq)
find_package(cxxopts)
target_link_libraries(controller cxxopts::cxxopts)
target_link_libraries(controller common) 
target_include_directories(controller PRIVATE ${CMAKE_SOURCE_DIR}/lib/UAV_common/header)
