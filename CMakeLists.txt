cmake_minimum_required(VERSION 3.24)
project(PSOMAB)

set(CMAKE_CXX_STANDARD 20)

include_directories(PSOMAB)
include_directories(third_party/multiprecision/include)
include_directories(third_party/eigen)


add_executable(PSOMAB
        pack_name/objects/arm/Arm.cpp
        pack_name/objects/arm/Arm.h
        pack_name/objects/look_up_tree/LUT.cpp
        pack_name/objects/look_up_tree/LUT.h
        pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.cpp
        pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h
        pack_name/util/RandomNumber.cpp
        pack_name/util/RandomNumber.h
        tests/main.cpp
        tests/problems/inventory.h
        pack_name/algo/particle_swarm_optimizer/PSO.cpp
        pack_name/algo/particle_swarm_optimizer/PSO.h
        pack_name/objects/solution/Solution.cpp
        pack_name/objects/solution/Solution.h)
