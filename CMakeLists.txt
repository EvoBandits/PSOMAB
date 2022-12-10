cmake_minimum_required(VERSION 3.24)
project(PSOMAB)

set(CMAKE_CXX_STANDARD 20)

include_directories(PSOMAB)
include_directories(third_party/boost_1_80_0)
include_directories(third_party/eigen)


add_executable(PSOMAB
        pack_name/PSOMAB/Arm.cpp
        pack_name/PSOMAB/Arm.h
        pack_name/PSOMAB/LUT.cpp
        pack_name/PSOMAB/LUT.h
        pack_name/PSOMAB/PSOMAB.cpp
        pack_name/PSOMAB/PSOMAB.h
        pack_name/PSOMAB/RandomNumber.cpp
        pack_name/PSOMAB/RandomNumber.h
        tests/main.cpp
        tests/problems/inventory.h)
