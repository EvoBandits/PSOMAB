#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include "../pack_name/algo/particle_swarm_optimizer_average_neighborhood/PSOAN.h"
#include "../pack_name/algo/particle_swarm_optimizer_equal_resampling/PSOER.h"
#include "../pack_name/algo/particle_swarm_optimizer_group_decision/PSOGD.h"
#include "../pack_name/algo/particle_swarm_optimizer_learning_automaton/LAPSO.h"
#include "../pack_name/algo/particle_swarm_optimizer_learning_automaton/PSOLA.h"
#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation/PSOOCBA.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation_adjusted/PSOOCBAA.h"
#include "../pack_name/algo/particle_swarm_optimizer_top_n_resampling/PSOERN.h"

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

void single_run(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity);
double multiple_runs(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity, int num_runs);

int max_simulation = 10000;
int num_particle = 50;
bool use_random_location_update = false;
bool cap_velocity = true;

std::string problems[] = {"tp1", "tp2", "styblinski-tang", "ackley", "inventory"};
std::string algos[] = {"psoan", "psoern", "psomab", "psogd", "lapso", "psoocbaa", "pso"};

int num_runs = 500;

void run(const std::string &problem, const std::string &algo) {
        std::cout << problem << " " << algo << std::endl;
        double mean_reward = multiple_runs(problem, algo, max_simulation, num_particle, use_random_location_update, cap_velocity, num_runs);
        std::cout << "Mean reward: " << mean_reward << std::endl;
        //write to file
        std::ofstream file(problem + "_" + algo + "_" + std::to_string(num_runs) + ".csv");
        file << num_runs << "," << mean_reward << std::endl;
        file.close();
}

int main() {
        std::vector<std::thread> threads;
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

void single_run(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity) {
        std::string time = std::to_string(std::time(nullptr));

        int dimension;
        Eigen::VectorXi lb;
        Eigen::VectorXi ub;
        std::function<double(Eigen::VectorXi, int)> opti_func;
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

        if (algo == "pso") {
                std::cout << "PSO" << std::endl;
                PSO pso_instance = PSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                pso_instance.optimize();

                for (auto &best_solution : pso_instance.best_solutions()) {
                        best_solution.print();
                }
                pso_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        } else if (algo == "psoan"){
                std::cout << "PSOAN" << std::endl;
                PSOAN psoan_instance = PSOAN(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoan_instance.optimize();

                for (auto &best_solution : psoan_instance.best_solutions()) {
                        best_solution.print();
                }
                psoan_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        } else if (algo == "psogd"){
                std::cout << "PSOGD" << std::endl;
                PSOGD psogd_instance = PSOGD(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psogd_instance.optimize();

                for (auto &best_solution : psogd_instance.best_solutions()) {
                        best_solution.print();
                }
                psogd_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        } else if (algo == "psoocba"){
                std::cout << "PSOOCBA" << std::endl;
                PSOOCBA psoocba_instance = PSOOCBA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoocba_instance.optimize();

                for (auto &best_solution : psoocba_instance.best_solutions()) {
                        best_solution.print();
                }
                psoocba_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        } else if (algo == "psoocbaa"){
                std::cout << "PSOOCBAA" << std::endl;
                PSOOCBAA psoocbaa_instance = PSOOCBAA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoocbaa_instance.optimize();

                for (auto &best_solution : psoocbaa_instance.best_solutions()) {
                        best_solution.print();
                }
                psoocbaa_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        } else if (algo == "psola"){
                std::cout << "PSOLA" << std::endl;
                PSOLA psola_instance = PSOLA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psola_instance.optimize();

                for (auto &best_solution : psola_instance.best_solutions()) {
                        best_solution.print();
                }
                psola_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        }
        else if (algo == "lapso"){
                std::cout << "LAPSO" << std::endl;
                LAPSO lapso_instance = LAPSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                lapso_instance.optimize();

                for (auto &best_solution : lapso_instance.best_solutions()) {
                        best_solution.print();
                }
                lapso_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        }
        else if (algo == "psomab"){
                std::cout << "PSOMAB:" << std::endl;
                PSOMAB psomab_instance = PSOMAB(opti_func, max_simulation, num_particle, lb, ub, dimension, use_random_location_update, cap_velocity);
                psomab_instance.optimize();

                for (auto &best_solution : psomab_instance.best_solutions()) {
                        best_solution.print();
                }
                psomab_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        }
        else if (algo == "psoern"){
                std::cout << "PSOERN:" << std::endl;
                PSOERN psoern_instance = PSOERN(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoern_instance.optimize();

                for (auto &best_solution : psoern_instance.best_solutions()) {
                        best_solution.print();
                }
                psoern_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        }
        else if (algo == "psoer"){
                std::cout << "PSOER:" << std::endl;
                PSOER psoer_instance = PSOER(num_particle, dimension, lb, ub, opti_func, max_simulation, 20, use_random_location_update, cap_velocity);
                psoer_instance.optimize();

                for (auto &best_solution : psoer_instance.best_solutions()) {
                        best_solution.print();
                }
                psoer_instance.memory_to_csv(algo + "_memory_" + time + ".csv");
        }
}

double multiple_runs(const std::string &problem, const std::string &algo, int max_simulation, int num_particle, bool use_random_location_update, bool cap_velocity, int num_runs) {
        double mean_reward = 0;

        int dimension;
        Eigen::VectorXi lb;
        Eigen::VectorXi ub;
        std::function<double(Eigen::VectorXi, int)> opti_func;
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
                std::cout << "PSO" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSO pso_instance = PSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        pso_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << pso_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += pso_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psoan") {
                std::cout << "PSOAN" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOAN psoan_instance = PSOAN(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        psoan_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psoan_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psoan_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psogd") {
                std::cout << "PSOGD" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOGD psogd_instance = PSOGD(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        psogd_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psogd_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psogd_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psoocba") {
                std::cout << "PSOOCBA" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOOCBA psoocba_instance = PSOOCBA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        psoocba_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psoocba_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psoocba_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psoocbaa") {
                std::cout << "PSOOCBAA" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOOCBAA psoocbaa_instance = PSOOCBAA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        psoocbaa_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psoocbaa_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psoocbaa_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psola") {
                std::cout << "PSOLA" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOLA psola_instance = PSOLA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        psola_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psola_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psola_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "lapso") {
                std::cout << "LAPSO" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        LAPSO lapso_instance = LAPSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        lapso_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << lapso_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += lapso_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psomab") {
                std::cout << "PSOMAB:" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOMAB psomab_instance = PSOMAB(opti_func, max_simulation, num_particle, lb, ub, dimension, use_random_location_update, cap_velocity);
                        psomab_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psomab_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psomab_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psoern") {
                std::cout << "PSOERN:" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOERN psoern_instance = PSOERN(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                        psoern_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psoern_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psoern_instance.best_solutions().back().true_func_val;
                }
        } else if (algo == "psoer") {
                std::cout << "PSOER:" << std::endl;
                for (int i = 0; i < num_runs; i++) {
                        PSOER psoer_instance = PSOER(num_particle, dimension, lb, ub, opti_func, max_simulation, 20, use_random_location_update, cap_velocity);
                        psoer_instance.optimize();
                        std::cout << "Run: " << i << " | best reward: " << psoer_instance.best_solutions().back().true_func_val << std::endl;
                        mean_reward += psoer_instance.best_solutions().back().true_func_val;
                }
        }

        return mean_reward / num_runs;
}