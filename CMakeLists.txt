cmake_minimum_required(VERSION 3.24)
project(PSOMAB)

set(CMAKE_CXX_STANDARD 20)

include_directories(PSOMAB)
include_directories(third_party/boost_1_80_0)
include_directories(third_party/eigen-3.4.0)

add_executable(PSOMAB
        PSOMAB/Arm.cpp
        PSOMAB/Arm.h
        PSOMAB/LUT.cpp
        PSOMAB/LUT.h
        PSOMAB/main.cpp
        PSOMAB/PSOMAB.cpp
        PSOMAB/PSOMAB.h)
