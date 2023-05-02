#include "../memo/algo/particle_swarm_optimizer/PSO.h"
#include "../memo/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.h"
#include "../memo/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h"
#include "../memo/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.h"
#include "../memo/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.h"
#include "../memo/algo/particle_swarm_optimizer/variants/learning_automaton/PSOLA.h"
#include "../memo/algo/particle_swarm_optimizer/variants/memory_enhanced/MEMOPSO.h"
#include "../memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.h"
#include "../memo/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.h"
#include "../memo/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <omp.h>
#include <thread>

#include "problems/ackley/ackley.h"
#include "problems/inventory/inventory.h"
#include "problems/styblinski-tang/styblinski-tang.h"
#include "problems/tp1/tp1.h"
#include "problems/tp2/tp2.h"

#include "../memo/objects/solution/Solution.h"

// test linter

auto single_run(const std::string &problem, const std::string &algo);
auto multiple_runs(const std::string &problem, const std::string &algo);
void run(const std::string &problem, const std::string &algo);

const int MAX_SIMULATIONS = 10000;
const int NUM_PARTICLES = 50;
const bool USE_RANDOM_LOCATION_UPDATE = false;
const bool CAP_VELOCITY = true;
const bool MEMORY = true;
const bool to_csv_ = true;

const std::array<std::string, 1> problems = {"inventory"};
const std::array<std::string, 1> algos = {"memopso"};

const int NUM_RUNS = 2;

auto main() -> int {
        auto threads = std::vector<std::thread>();
        for (const auto &problem : problems) {
                for (const auto &algo : algos) {
                        threads.emplace_back(run, problem, algo);
                }
        }

        for (auto &thread : threads) {
                thread.join();
        }

        return 0;
}

auto single_run(const std::string &problem, const std::string &algo) {
        std::string time = std::to_string(std::time(nullptr));

        std::stringstream filename_stream("");
        filename_stream << problem << "_" << algo << "_memory_" << time << ".csv";

        int dimension = 2;
        Eigen::VectorXi lb(dimension);
        Eigen::VectorXi ub(dimension);
        auto opti_func = inventory;
        if (problem == "inventory") {
                dimension = inventory_dim;
                lb = inventory_lb;
                ub = inventory_ub;
                opti_func = inventory;
        } else if (problem == "tp1") {
                dimension = tp1_dim;
                lb = tp1_lb;
                ub = tp1_ub;
                opti_func = reinterpret_cast<double (*)(Eigen::VectorXi, int)>(tp1);
        } else if (problem == "tp2") {
                dimension = tp2_dim;
                lb = tp2_lb;
                ub = tp2_ub;
                opti_func = reinterpret_cast<double (*)(Eigen::VectorXi, int)>(tp2);
        } else if (problem == "ackley") {
                dimension = ackley_dim;
                lb = ackley_lb;
                ub = ackley_ub;
                opti_func = reinterpret_cast<double (*)(Eigen::VectorXi, int)>(ackley);
        } else if (problem == "styblinski-tang") {
                dimension = styblinski_tang_dim;
                lb = styblinski_tang_lb;
                ub = styblinski_tang_ub;
                opti_func = reinterpret_cast<double (*)(Eigen::VectorXi, int)>(styblinski_tang);
        }

        std::cout << std::fixed;
        std::cout << std::setprecision(3);

        if (algo == "pso") {
                std::cout << "PSO ";
                PSO pso_instance = PSO(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                pso_instance.optimize();
                if (MEMORY) {
                        pso_instance.memory_to_csv(filename_stream.str());
                }
                return pso_instance.best_solutions();
        }
        if (algo == "psoan") {
                std::cout << "PSOAN ";
                PSOAN psoan_instance = PSOAN(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoan_instance.optimize();
                if (MEMORY) {
                        psoan_instance.memory_to_csv(filename_stream.str());
                }
                return psoan_instance.best_solutions();
        }
        if (algo == "psogd") {
                std::cout << "PSOGD ";
                PSOGD psogd_instance = PSOGD(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psogd_instance.optimize();
                if (MEMORY) {
                        psogd_instance.memory_to_csv(filename_stream.str());
                }
                return psogd_instance.best_solutions();
        }
        if (algo == "psoocba") {
                std::cout << "PSOOCBA ";
                PSOOCBA psoocba_instance = PSOOCBA(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoocba_instance.optimize();
                if (MEMORY) {
                        psoocba_instance.memory_to_csv(filename_stream.str());
                }
                return psoocba_instance.best_solutions();
        }
        if (algo == "psoocbaa") {
                std::cout << "PSOOCBAA ";
                PSOOCBAA psoocbaa_instance = PSOOCBAA(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoocbaa_instance.optimize();
                if (MEMORY) {
                        psoocbaa_instance.memory_to_csv(filename_stream.str());
                }
                return psoocbaa_instance.best_solutions();
        }
        if (algo == "psola") {
                std::cout << "PSOLA ";
                PSOLA psola_instance = PSOLA(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psola_instance.optimize();
                if (MEMORY) {
                        psola_instance.memory_to_csv(filename_stream.str());
                }
                return psola_instance.best_solutions();
        }
        if (algo == "lapso") {
                std::cout << "LAPSO ";
                LAPSO lapso_instance = LAPSO(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                lapso_instance.optimize();
                if (MEMORY) {
                        lapso_instance.memory_to_csv(filename_stream.str());
                }
                return lapso_instance.best_solutions();
        }
        if (algo == "memopso") {
                std::cout << "MEMOPSO: ";
                MEMOPSO memopso_instance = MEMOPSO(opti_func, lb, ub, dimension, MAX_SIMULATIONS, NUM_PARTICLES, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                memopso_instance.optimize();
                if (MEMORY) {
                        memopso_instance.memory_to_csv(filename_stream.str());
                }
                return memopso_instance.best_solutions();
        }
        if (algo == "psoern") {
                std::cout << "PSOERN: ";
                PSOERN psoern_instance = PSOERN(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoern_instance.optimize();
                if (MEMORY) {
                        psoern_instance.memory_to_csv(filename_stream.str());
                }
                return psoern_instance.best_solutions();
        }
        if (algo == "psoer") {
                std::cout << "PSOER: ";
                PSOER psoer_instance = PSOER(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, 20, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoer_instance.optimize();
                if (MEMORY) {
                        psoer_instance.memory_to_csv(filename_stream.str());
                }
                return psoer_instance.best_solutions();
        }
}

auto multiple_runs(const std::string &problem, const std::string &algo) {
        auto all_solutions = std::array<std::vector<solution>, NUM_RUNS>{};

        for (int i = 0; i < NUM_RUNS; ++i) {
                auto best_solutions = single_run(problem, algo);
                all_solutions[i] = best_solutions;
                double reward = 0.0;
                if (!best_solutions.empty()) {
                        reward = best_solutions.back().true_func_val;
                }
                std::cout << "Run: " << i << " | best reward: " << reward << std::endl;
        }

        return all_solutions;
}

void run(const std::string &problem, const std::string &algo) {
        std::cout << problem << " " << algo << std::endl;
        auto solutions = multiple_runs(problem, algo);
        if (to_csv_) {
                std::string time = std::to_string(std::time(nullptr));
                std::stringstream file_name("");
                file_name << problem << "_" << algo << "_" << time << ".csv";
                std::ofstream file(file_name.str());
                file << std::fixed;
                file << std::setprecision(3);
                for (const auto &solution : solutions) {
                        for (const auto &s : solution) {
                                file << s.true_func_val << ",";
                        }
                        file << "\n";
                }
        }
}
