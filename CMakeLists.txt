cmake_minimum_required(VERSION 3.22)
project(MEMO)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
if (BUILD_TESTS)
    set(CMAKE_CXX_FLAGS "--coverage")
endif ()

include_directories(third_party/eigen)

if (BUILD_TESTS)
    add_subdirectory(third_party/Catch2)
endif ()

include(FetchContent)
FetchContent_Declare(cpr GIT_REPOSITORY https://github.com/libcpr/cpr.git
        GIT_TAG 1.10.5)
set(BUILD_SHARED_LIBS OFF)
FetchContent_MakeAvailable(cpr)

FetchContent_Declare(nlohmann_json GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3)
FetchContent_MakeAvailable(nlohmann_json)

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
        memo/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h
        tests/MLflowLogger.cpp
        tests/MLflowLogger.h
        tests/SecretManagement.cpp
        tests/SecretManagement.h
        tests/problem_parameter.h
        tests/util/ProgressBar.cpp
        tests/util/ProgressBar.h
        tests/problems/eggholder/eggholder.cpp
        tests/problems/eggholder/eggholder.h
        tests/problems/michalewicz_ackley/michalewicz_ackley.cpp
        tests/problems/michalewicz_ackley/michalewicz_ackley.h
        tests/problems/michalewicz_sphere/michalewicz_sphere.cpp
        tests/problems/michalewicz_sphere/michalewicz_sphere.h)

if (BUILD_TESTS)
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
endif ()

target_link_libraries(MEMO PRIVATE cpr::cpr)
target_link_libraries(MEMO PRIVATE nlohmann_json::nlohmann_json)

if (BUILD_TESTS)
    include(CTest)
    include(Catch)
    catch_discover_tests(tests)
endif ()
