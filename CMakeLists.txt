cmake_minimum_required(VERSION 3.24)
project(PSOMAB)

set(CMAKE_CXX_STANDARD 20)

include_directories(src)
include_directories(third_party/boost_1_80_0)

add_executable(PSOMAB
        src/Arm.cpp
        src/Arm.h
        src/LUT.cpp
        src/LUT.h
        src/main.cpp
        src/PSOMAB.cpp
        src/PSOMAB.h)
