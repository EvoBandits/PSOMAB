cmake_minimum_required(VERSION 3.24)
project(PSOMAB)

set(CMAKE_CXX_STANDARD 20)

include_directories(PSOMAB)
include_directories(third_party/multiprecision/include)
include_directories(third_party/eigen)


add_executable(PSOMAB
        pack_name/Arm.cpp
        pack_name/Arm.h
        pack_name/LUT.cpp
        pack_name/LUT.h
        pack_name/algo/PSOMAB/PSOMAB.cpp
        pack_name/algo/PSOMAB/PSOMAB.h
        pack_name/RandomNumber.cpp
        pack_name/RandomNumber.h
        tests/main.cpp
        tests/problems/inventory.h
        pack_name/algo/PSO/PSO.cpp
        pack_name/algo/PSO/PSO.h
        pack_name/Solution.cpp
        pack_name/Solution.h)
