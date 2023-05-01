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

auto single_run(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity);
auto multiple_runs(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity, int num_runs, bool memory);
void run(const std::string &problem, const std::string &algo);

const int MAX_SIMULATIONS = 10000;
const int NUM_PARTICLES = 50;
const bool use_random_location_update_ = false;
const bool cap_velocity_ = true;
const bool memory_ = false;
const bool to_csv_ = true;

const std::array<std::string, 1> problems = {"ackley"};
const std::array<std::string, 1> algos = {"memopso"};

const int NUM_RUNS = 1;

int main() {
        std::vector<std::thread> threads{};
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

auto single_run(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity, bool memory) {
        std::string time = std::to_string(std::time(nullptr));

        int dimension = 0;
        Eigen::VectorXi lb(dimension);
        Eigen::VectorXi ub(dimension);
        std::function<double(Eigen::VectorXi, int)> opti_func = nullptr;
        if (problem == "inventory") {
                dimension = inventory_dim;
                lb = inventory_lb;
                ub = inventory_ub;
                opti_func = inventory;
        } else if (problem == "tp1") {
                dimension = tp1_dim;
                lb = tp1_lb;
                ub = tp1_ub;
                opti_func = tp1;
        } else if (problem == "tp2") {
                dimension = tp2_dim;
                lb = tp2_lb;
                ub = tp2_ub;
                opti_func = tp2;
        } else if (problem == "ackley") {
                dimension = ackley_dim;
                lb = ackley_lb;
                ub = ackley_ub;
                opti_func = ackley;
        } else if (problem == "styblinski-tang") {
                dimension = styblinski_tang_dim;
                lb = styblinski_tang_lb;
                ub = styblinski_tang_ub;
                opti_func = styblinski_tang;
        }

        std::cout << std::fixed;
        std::cout << std::setprecision(3);

        if (algo == "pso") {
                std::cout << "PSO ";
                PSO pso_instance = PSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                pso_instance.optimize();
                if (memory) {
                        pso_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return pso_instance.best_solutions();
        } else if (algo == "psoan") {
                std::cout << "PSOAN ";
                PSOAN psoan_instance = PSOAN(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoan_instance.optimize();
                if (memory) {
                        psoan_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return psoan_instance.best_solutions();
        } else if (algo == "psogd") {
                std::cout << "PSOGD ";
                PSOGD psogd_instance = PSOGD(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psogd_instance.optimize();
                if (memory) {
                        psogd_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return psogd_instance.best_solutions();
        } else if (algo == "psoocba") {
                std::cout << "PSOOCBA ";
                PSOOCBA psoocba_instance = PSOOCBA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoocba_instance.optimize();
                if (memory) {
                        psoocba_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return psoocba_instance.best_solutions();
        } else if (algo == "psoocbaa") {
                std::cout << "PSOOCBAA ";
                PSOOCBAA psoocbaa_instance = PSOOCBAA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoocbaa_instance.optimize();
                if (memory) {
                        psoocbaa_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return psoocbaa_instance.best_solutions();
        } else if (algo == "psola") {
                std::cout << "PSOLA ";
                PSOLA psola_instance = PSOLA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psola_instance.optimize();
                if (memory) {
                        psola_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return psola_instance.best_solutions();
        } else if (algo == "lapso") {
                std::cout << "LAPSO ";
                LAPSO lapso_instance = LAPSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                lapso_instance.optimize();
                if (memory) {
                        lapso_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return lapso_instance.best_solutions();
        } else if (algo == "memopso") {
                std::cout << "MEMOPSO: ";
                MEMOPSO memopso_instance = MEMOPSO(opti_func, lb, ub, dimension, max_simulation, num_particle, use_random_location_update, cap_velocity);
                memopso_instance.optimize();
                if (memory) {
                        memopso_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return memopso_instance.best_solutions();
        } else if (algo == "psoern") {
                std::cout << "PSOERN: ";
                PSOERN psoern_instance = PSOERN(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoern_instance.optimize();
                if (memory) {
                        psoern_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return psoern_instance.best_solutions();
        } else if (algo == "psoer") {
                std::cout << "PSOER: ";
                PSOER psoer_instance = PSOER(num_particle, dimension, lb, ub, opti_func, max_simulation, 20, use_random_location_update, cap_velocity);
                psoer_instance.optimize();
                if (memory) {
                        psoer_instance.memory_to_csv(problem + "_" + algo + "_memory_" + time + ".csv");
                }
                return psoer_instance.best_solutions();
        }
}

auto multiple_runs(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity, int num_runs, bool memory) {
        std::vector<std::vector<solution>> all_solutions;

        for (int i = 0; i < num_runs; i++) {
                std::vector<solution> best_solutions = single_run(problem, algo, max_simulation, num_particle, use_random_location_update, cap_velocity, memory);
                all_solutions.push_back(best_solutions);
                double reward = best_solutions.back().true_func_val;
                std::cout << "Run: " << i << " | best reward: " << reward << std::endl;
        }

        return all_solutions;
}

void run(const std::string &problem, const std::string &algo) {
        std::cout << problem << " " << algo << std::endl;
        std::vector<std::vector<solution>> solutions = multiple_runs(problem, algo, MAX_SIMULATIONS, NUM_PARTICLES, use_random_location_update_, cap_velocity_, NUM_RUNS, memory_);
        if (to_csv_) {
                std::string time = std::to_string(std::time(nullptr));
                std::string file_name = problem + "_" + algo + "_" + time + ".csv";
                std::ofstream file(file_name);
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
