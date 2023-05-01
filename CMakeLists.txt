cmake_minimum_required(VERSION 3.22)
project(MEMO)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "--coverage")

include_directories(third_party/eigen)
add_subdirectory(third_party/Catch2)

add_executable(MEMO
        memo/objects/arm/Arm.cpp
        memo/objects/arm/Arm.h
        memo/algo/particle_swarm_optimizer/variants/memory_enhanced/MEMOPSO.cpp
        memo/algo/particle_swarm_optimizer/variants/memory_enhanced/MEMOPSO.h
        memo/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.cpp
        memo/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.h
        memo/util/RandomNumber.cpp
        memo/util/RandomNumber.h
        tests/main.cpp
        tests/problems/inventory/inventory.h
        tests/problems/inventory/inventory.cpp
        memo/algo/particle_swarm_optimizer/PSO.cpp
        memo/algo/particle_swarm_optimizer/PSO.h
        memo/objects/solution/Solution.cpp
        memo/objects/solution/Solution.h
        memo/util/SolutionCodeCalculation.h
        tests/problems/ackley/ackley.h
        tests/problems/ackley/ackley.cpp
        tests/problems/tp1/tp1.cpp
        tests/problems/tp1/tp1.h
        tests/problems/tp2/tp2.cpp
        tests/problems/tp2/tp2.h
        tests/problems/styblinski-tang/styblinski-tang.cpp
        tests/problems/styblinski-tang/styblinski-tang.h
        memo/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.h
        memo/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.cpp
        memo/util/SortIndices.h memo/util/SortIndices.cpp
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.cpp
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.h
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.h
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.cpp
        memo/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.cpp
        memo/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.h
        memo/algo/particle_swarm_optimizer/variants/learning_automaton/PSOLA.cpp
        memo/util/SmartRounding.cpp memo/util/SmartRounding.h
        memo/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.cpp
        memo/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.h
        memo/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.cpp
        memo/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h)


add_executable(tests tests/test.cpp
        memo/objects/arm/Arm.cpp
        memo/objects/arm/Arm.h
        memo/algo/particle_swarm_optimizer/variants/memory_enhanced/MEMOPSO.cpp
        memo/algo/particle_swarm_optimizer/variants/memory_enhanced/MEMOPSO.h
        memo/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.cpp
        memo/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.h
        memo/util/RandomNumber.cpp
        memo/util/RandomNumber.h
        memo/algo/particle_swarm_optimizer/PSO.cpp
        memo/algo/particle_swarm_optimizer/PSO.h
        memo/objects/solution/Solution.cpp
        memo/objects/solution/Solution.h
        memo/util/SolutionCodeCalculation.h
        memo/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.h
        memo/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.cpp
        memo/util/SortIndices.h memo/util/SortIndices.cpp
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.cpp
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.h
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.h
        memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.cpp
        memo/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.cpp
        memo/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.h
        memo/algo/particle_swarm_optimizer/variants/learning_automaton/PSOLA.cpp
        memo/util/SmartRounding.cpp memo/util/SmartRounding.h
        memo/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.cpp
        memo/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.h
        memo/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.cpp
        memo/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h)

target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(tests)
