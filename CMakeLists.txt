cmake_minimum_required(VERSION 3.5)
project(controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

file(GLOB_RECURSE SOURCES ${SOURCE_DIR}/*.cpp)

include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR})

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")

add_executable(controller ${SOURCES})
set_property(TARGET controller PROPERTY CXX_STANDARD 20)
target_include_directories(controller PUBLIC include)
target_compile_features(controller PUBLIC cxx_std_20)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(controller Eigen3::Eigen)
find_package(cppzmq)
target_link_libraries(controller cppzmq)
find_package(cxxopts)
target_link_libraries(controller cxxopts::cxxopts)
