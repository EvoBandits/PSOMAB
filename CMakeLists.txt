cmake_minimum_required(VERSION 3.24)
project(PSOMAB)

set(CMAKE_CXX_STANDARD 20)

include_directories(PSOMAB)
include_directories(third_party/multiprecision/include)
include_directories(third_party/eigen)


add_executable(PSOMAB
        pack_name/objects/arm/Arm.cpp
        pack_name/objects/arm/Arm.h
        pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.cpp
        pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h
        pack_name/algo/particle_swarm_optimizer_average_neighborhood/PSOAN.cpp
        pack_name/algo/particle_swarm_optimizer_average_neighborhood/PSOAN.h
        pack_name/util/RandomNumber.cpp
        pack_name/util/RandomNumber.h
        tests/main.cpp
        tests/problems/inventory/inventory.h
        tests/problems/inventory/inventory.cpp
        pack_name/algo/particle_swarm_optimizer/PSO.cpp
        pack_name/algo/particle_swarm_optimizer/PSO.h
        pack_name/objects/solution/Solution.cpp
        pack_name/objects/solution/Solution.h
        pack_name/util/SolutionCodeCalculation.h
        tests/problems/ackley/ackley.h
        tests/problems/ackley/ackley.cpp
        tests/problems/tp1/tp1.cpp
        tests/problems/tp1/tp1.h
        tests/problems/tp2/tp2.h
        tests/problems/tp2/tp2.cpp pack_name/algo/particle_swarm_optimizer_group_decision/PSOGD.h pack_name/algo/particle_swarm_optimizer_group_decision/PSOGD.cpp pack_name/util/SortIndices.h pack_name/util/SortIndices.cpp pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation/PSOOCBA.cpp pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation/PSOOCBA.h pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation_adjusted/PSOOCBAA.h pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation_adjusted/PSOOCBAA.cpp "pack_name/algo/particle_swarm_optimizer_learning_automaton/LAPSO.cpp" "pack_name/algo/particle_swarm_optimizer_learning_automaton/LAPSO.h" "pack_name/algo/particle_swarm_optimizer_learning_automaton/PSOLA.cpp" pack_name/util/SmartRounding.cpp pack_name/util/SmartRounding.h pack_name/algo/particle_swarm_optimizer_equal_resampling/PSOER.cpp pack_name/algo/particle_swarm_optimizer_equal_resampling/PSOER.h)
