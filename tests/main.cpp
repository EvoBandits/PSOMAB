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
#include <thread>

#include <nlohmann/json.hpp>

#include "problems/ackley/ackley.h"
#include "problems/inventory/inventory.h"
#include "problems/styblinski-tang/styblinski-tang.h"
#include "problems/tp1/tp1.h"
#include "problems/tp2/tp2.h"

#include "MLflowLogger.h"
#include "SecretManagement.h"

#include "../memo/objects/solution/Solution.h"

nlohmann::json config;
std::vector<std::string> algos;

auto single_run(const std::string &problem, const std::string &algo);
auto multiple_runs(const std::string &problem, const std::string &algo);
void run(const std::string &problem, const std::string &algo);

std::string experiment_id;
std::unique_ptr<MLflowLogger> mlflow_logger;

int dimension = 2;// dummy value
Eigen::VectorXi lb(dimension);
Eigen::VectorXi ub(dimension);
double step_size = 0.01;// dummy value

double noise_level = 0.5;// dummy value

auto main() -> int {
        config = nlohmann::json::parse(std::ifstream("config.json"));

        for (const auto &algo : config["algos"]) {
                algos.push_back(algo);
        }
        const bool track_experiments = config["track_experiments"];

        const std::string test_problem = config["test_problem"];
        const std::string experiment_name = config["experiment_name"];

        if (track_experiments) {
                mlflow_logger = std::make_unique<MLflowLogger>();
                experiment_id = mlflow_logger->get_or_create_experiment(experiment_name);
        }

        auto threads = std::vector<std::thread>();
        for (const auto &algo : algos) {
                threads.emplace_back(run, test_problem, algo);
        }

        for (auto &thread : threads) {
                thread.join();
        }

        return 0;
}

auto single_run(const std::string &problem, const std::string &algo) {
        const int MAX_SIMULATIONS = config["MAX_SIMULATIONS"];
        const int NUM_PARTICLES = config["NUM_PARTICLES"];
        const bool USE_RANDOM_LOCATION_UPDATE = config["USE_RANDOM_LOCATION_UPDATE"];
        const bool CAP_VELOCITY = config["CAP_VELOCITY"];
        const bool MEMORY = config["MEMORY"];
        const double W = config["W"];
        const double C1 = config["C1"];
        const double C2 = config["C2"];
        std::string time = std::to_string(std::time(nullptr));

        std::stringstream filename_stream("");
        filename_stream << problem << "_" << algo << "_memory_" << time << ".csv";

        auto opti_func = inventory;
        if (problem == "inventory") {
                dimension = inventory_dim;
                lb = inventory_lb;
                ub = inventory_ub;
                step_size = 1;

                noise_level = inventory_noise_level;

                opti_func = inventory;
        } else if (problem == "ackley") {
                dimension = ackley_dim;
                lb = ackley_lb;
                ub = ackley_ub;
                step_size = ackley_step_size;

                noise_level = ackley_noise_level;

                opti_func = reinterpret_cast<double (*)(Eigen::VectorXi, bool)>(ackley);
        } else if (problem == "styblinski-tang") {
                dimension = styblinski_tang_dim;
                lb = styblinski_tang_lb;
                ub = styblinski_tang_ub;
                step_size = styblinski_tang_step_size;

                noise_level = styblinski_tang_noise_level;

                opti_func = reinterpret_cast<double (*)(Eigen::VectorXi, bool)>(styblinski_tang);
        }

        std::cout << std::fixed;
        std::cout << std::setprecision(3);

        if (algo == "pso") {
                std::cout << "PSO ";
                PSO pso_instance = PSO(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                pso_instance.optimize();
                if (MEMORY) {
                        pso_instance.memory_to_csv(filename_stream.str());
                }
                return pso_instance.best_solutions();
        }
        if (algo == "psoan") {
                std::cout << "PSOAN ";
                PSOAN psoan_instance = PSOAN(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoan_instance.optimize();
                if (MEMORY) {
                        psoan_instance.memory_to_csv(filename_stream.str());
                }
                return psoan_instance.best_solutions();
        }
        if (algo == "psogd") {
                std::cout << "PSOGD ";
                PSOGD psogd_instance = PSOGD(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psogd_instance.optimize();
                if (MEMORY) {
                        psogd_instance.memory_to_csv(filename_stream.str());
                }
                return psogd_instance.best_solutions();
        }
        if (algo == "psoocba") {
                std::cout << "PSOOCBA ";
                PSOOCBA psoocba_instance = PSOOCBA(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoocba_instance.optimize();
                if (MEMORY) {
                        psoocba_instance.memory_to_csv(filename_stream.str());
                }
                return psoocba_instance.best_solutions();
        }
        if (algo == "psoocbaa") {
                std::cout << "PSOOCBAA ";
                PSOOCBAA psoocbaa_instance = PSOOCBAA(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoocbaa_instance.optimize();
                if (MEMORY) {
                        psoocbaa_instance.memory_to_csv(filename_stream.str());
                }
                return psoocbaa_instance.best_solutions();
        }
        if (algo == "psola") {
                std::cout << "PSOLA ";
                PSOLA psola_instance = PSOLA(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psola_instance.optimize();
                if (MEMORY) {
                        psola_instance.memory_to_csv(filename_stream.str());
                }
                return psola_instance.best_solutions();
        }
        if (algo == "lapso") {
                std::cout << "LAPSO ";
                LAPSO lapso_instance = LAPSO(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                lapso_instance.optimize();
                if (MEMORY) {
                        lapso_instance.memory_to_csv(filename_stream.str());
                }
                return lapso_instance.best_solutions();
        }
        if (algo == "memopso") {
                std::cout << "MEMOPSO: ";
                MEMOPSO memopso_instance = MEMOPSO(opti_func, lb, ub, dimension, MAX_SIMULATIONS, W, C1, C2, NUM_PARTICLES, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                memopso_instance.optimize();
                if (MEMORY) {
                        memopso_instance.memory_to_csv(filename_stream.str());
                }
                return memopso_instance.best_solutions();
        }
        if (algo == "psoern") {
                std::cout << "PSOERN: ";
                PSOERN psoern_instance = PSOERN(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoern_instance.optimize();
                if (MEMORY) {
                        psoern_instance.memory_to_csv(filename_stream.str());
                }
                return psoern_instance.best_solutions();
        }
        if (algo == "psoer") {
                std::cout << "PSOER: ";
                PSOER psoer_instance = PSOER(NUM_PARTICLES, dimension, lb, ub, opti_func, MAX_SIMULATIONS, 20, W, C1, C2, USE_RANDOM_LOCATION_UPDATE, CAP_VELOCITY);
                psoer_instance.optimize();
                if (MEMORY) {
                        psoer_instance.memory_to_csv(filename_stream.str());
                }
                return psoer_instance.best_solutions();
        }
}

auto multiple_runs(const std::string &problem, const std::string &algo) {
        const int NUM_RUNS = config["NUM_RUNS"];
        std::vector<std::vector<solution>> all_solutions(NUM_RUNS);

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
        const int MAX_SIMULATIONS = config["MAX_SIMULATIONS"];
        const int NUM_PARTICLES = config["NUM_PARTICLES"];
        const bool USE_RANDOM_LOCATION_UPDATE = config["USE_RANDOM_LOCATION_UPDATE"];
        const bool CAP_VELOCITY = config["CAP_VELOCITY"];
        const int NUM_RUNS = config["NUM_RUNS"];
        const bool to_csv_ = config["to_csv"];

        std::cout << problem << " " << algo << std::endl;
        auto solutions = multiple_runs(problem, algo);
        // log average true func value
        double avg_true_func_val = 0.0;
        for (const auto &solution : solutions) {
                avg_true_func_val += solution.back().true_func_val;
        }
        avg_true_func_val /= NUM_RUNS;

        std::string run_id;
        if (!experiment_id.empty()) {
                run_id = mlflow_logger->start_run(experiment_id);

                mlflow_logger->log_param(run_id, "problem", problem);
                mlflow_logger->log_param(run_id, "dimension", std::to_string(dimension));
                mlflow_logger->log_param(run_id, "lb", std::to_string(lb(0) * step_size));
                mlflow_logger->log_param(run_id, "ub", std::to_string(ub(0) * step_size));
                mlflow_logger->log_param(run_id, "step_size", std::to_string(step_size));
                mlflow_logger->log_param(run_id, "noise_level", std::to_string(noise_level));

                mlflow_logger->log_param(run_id, "algorithm", algo);

                mlflow_logger->log_param(run_id, "runs", std::to_string(NUM_RUNS));
                mlflow_logger->log_param(run_id, "num_particles", std::to_string(NUM_PARTICLES));
                mlflow_logger->log_param(run_id, "max_simulations", std::to_string(MAX_SIMULATIONS));
                mlflow_logger->log_param(run_id, "use_random_location_update", std::to_string(USE_RANDOM_LOCATION_UPDATE));
                mlflow_logger->log_param(run_id, "cap_velocity", std::to_string(CAP_VELOCITY));

                mlflow_logger->log_metric(run_id, "avg_true_func_val", avg_true_func_val);
        }
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
                file.close();
                if (!experiment_id.empty()) {
                        mlflow_logger->upload_artifact(run_id, file_name.str());
                }
        }
}
