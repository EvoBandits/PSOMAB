cmake_minimum_required(VERSION 3.22)
project(PSOMAB)

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

add_executable(PSOMAB
        src/objects/arm/Arm.cpp
        src/objects/arm/Arm.h
        src/algo/particle_swarm_optimizer/variants/memory_enhanced/PSOMAB.cpp
        src/algo/particle_swarm_optimizer/variants/memory_enhanced/PSOMAB.h
        src/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.cpp
        src/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.h
        src/util/RandomNumber.cpp
        src/util/RandomNumber.h
        tests/main.cpp
        tests/problems/inventory/inventory.h
        tests/problems/inventory/inventory.cpp
        src/algo/particle_swarm_optimizer/PSO.cpp
        src/algo/particle_swarm_optimizer/PSO.h
        src/objects/solution/Solution.cpp
        src/objects/solution/Solution.h
        src/util/SolutionCodeCalculation.h
        tests/problems/f1_rosenbrock/f1_rosenbrock.h
        tests/problems/f1_rosenbrock/f1_rosenbrock.cpp
        tests/problems/f2_six_hump_camel_back/f2_six_hump_camel_back.h
        tests/problems/f2_six_hump_camel_back/f2_six_hump_camel_back.cpp
        tests/problems/f3_elliptic/f3_elliptic.h
        tests/problems/f3_elliptic/f3_elliptic.cpp
        tests/problems/f4_ackley/f4_ackley.h
        tests/problems/f4_ackley/f4_ackley.cpp
        tests/problems/two_agent_supply_chain/two_agent_supply_chain.cpp
        tests/problems/two_agent_supply_chain/two_agent_supply_chain.h
        tests/problems/four_agent_supply_chain/four_agent_supply_chain.cpp
        tests/problems/four_agent_supply_chain/four_agent_supply_chain.h
        tests/problems/styblinski-tang/styblinski-tang.cpp
        tests/problems/styblinski-tang/styblinski-tang.h
        src/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.h
        src/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.cpp
        src/util/SortIndices.h src/util/SortIndices.cpp
        src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.cpp
        src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.h
        src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.h
        src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.cpp
        src/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.cpp
        src/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.h
        src/algo/particle_swarm_optimizer/variants/learning_automaton/PSOLA.cpp
        src/util/SmartRounding.cpp src/util/SmartRounding.h
        src/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.cpp
        src/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.h
        src/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.cpp
        src/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h
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
            src/objects/arm/Arm.cpp
            src/objects/arm/Arm.h
            src/algo/particle_swarm_optimizer/variants/memory_enhanced/PSOMAB.cpp
            src/algo/particle_swarm_optimizer/variants/memory_enhanced/PSOMAB.h
            src/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.cpp
            src/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.h
            src/util/RandomNumber.cpp
            src/util/RandomNumber.h
            src/algo/particle_swarm_optimizer/PSO.cpp
            src/algo/particle_swarm_optimizer/PSO.h
            src/objects/solution/Solution.cpp
            src/objects/solution/Solution.h
            src/util/SolutionCodeCalculation.h
            src/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.h
            src/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.cpp
            src/util/SortIndices.h src/util/SortIndices.cpp
            src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.cpp
            src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.h
            src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.h
            src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.cpp
            src/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.cpp
            src/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.h
            src/algo/particle_swarm_optimizer/variants/learning_automaton/PSOLA.cpp
            src/util/SmartRounding.cpp src/util/SmartRounding.h
            src/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.cpp
            src/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.h
            src/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.cpp
            src/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h)

    target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
endif ()

target_link_libraries(PSOMAB PRIVATE cpr::cpr)
target_link_libraries(PSOMAB PRIVATE nlohmann_json::nlohmann_json)

if (BUILD_TESTS)
    include(CTest)
    include(Catch)
    catch_discover_tests(tests)
endif ()
