cmake_minimum_required(VERSION 3.24)
project(PSOMAB)

set(CMAKE_CXX_STANDARD 20)

include_directories(PSOMAB)
include_directories(third_party/multiprecision/include)
include_directories(third_party/eigen)


add_executable(PSOMAB
        pack_name/objects/arm/Arm.cpp
        pack_name/objects/arm/Arm.h
        pack_name/algo/particle_swarm_optimizer/variants/memory_enhanced/MEMOPSO.cpp
        pack_name/algo/particle_swarm_optimizer/variants/memory_enhanced/MEMOPSO.h
        pack_name/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.cpp
        pack_name/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.h
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
        tests/problems/tp2/tp2.cpp
        tests/problems/tp2/tp2.h
        tests/problems/styblinski-tang/styblinski-tang.cpp
        tests/problems/styblinski-tang/styblinski-tang.h
        pack_name/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.h
        pack_name/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.cpp
        pack_name/util/SortIndices.h pack_name/util/SortIndices.cpp
        pack_name/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.cpp
        pack_name/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.h
        pack_name/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.h
        pack_name/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.cpp
        pack_name/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.cpp
        pack_name/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.h
        pack_name/algo/particle_swarm_optimizer/variants/learning_automaton/PSOLA.cpp
        pack_name/util/SmartRounding.cpp pack_name/util/SmartRounding.h
        pack_name/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.cpp
        pack_name/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.h
        pack_name/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.cpp
        pack_name/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h)
