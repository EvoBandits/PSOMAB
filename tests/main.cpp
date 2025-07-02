#include "../src/algo/particle_swarm_optimizer/PSO.h"
#include "../src/algo/particle_swarm_optimizer/variants/average_neighborhood/PSOAN.h"
#include "../src/algo/particle_swarm_optimizer/variants/equal_resampling/PSOER.h"
#include "../src/algo/particle_swarm_optimizer/variants/group_decision/PSOGD.h"
#include "../src/algo/particle_swarm_optimizer/variants/learning_automaton/LAPSO.h"
#include "../src/algo/particle_swarm_optimizer/variants/learning_automaton/PSOLA.h"
#include "../src/algo/particle_swarm_optimizer/variants/memory_enhanced/PSOMAB.h"
#include "../src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation/PSOOCBA.h"
#include "../src/algo/particle_swarm_optimizer/variants/optimal_computing_budget_allocation_adjusted/PSOOCBAA.h"
#include "../src/algo/particle_swarm_optimizer/variants/top_n_resampling/PSOERN.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <thread>

#include <nlohmann/json.hpp>

#include "problem_parameter.h"
#include "problems/eggholder/eggholder.h"
#include "problems/f1_rosenbrock/f1_rosenbrock.h"
#include "problems/f2_six_hump_camel_back/f2_six_hump_camel_back.h"
#include "problems/f3_elliptic/f3_elliptic.h"
#include "problems/f4_ackley/f4_ackley.h"
#include "problems/inventory/inventory.h"
#include "problems/michalewicz_ackley/michalewicz_ackley.h"
#include "problems/michalewicz_sphere/michalewicz_sphere.h"
#include "problems/styblinski-tang/styblinski-tang.h"
#include "problems/tp1/tp1.h"
#include "problems/tp2/tp2.h"

#include "MLflowLogger.h"
#include "SecretManagement.h"

#include "util/ProgressBar.h"

#include "../src/objects/solution/Solution.h"

auto single_run(const std::string &problem, const std::string &algo, int num_particles, bool use_random_location_update, bool cap_velocity);
auto multiple_runs(const std::string &problem, const std::string &algo, int num_particles, bool use_random_location_update, bool cap_velocity);
void run_config(const std::string &problem, const std::string &algo, int num_particles, bool use_random_location_update, bool cap_velocity);
void run_all_configs(const std::string &problem);
void parse_config();
void parse_lb_ub();
void print_info();

nlohmann::json config;

std::vector<std::string> algos;
std::vector<int> nums_particles;
std::vector<bool> use_random_location_updates;
std::vector<bool> cap_velocities;

std::vector<int> dimensions;
std::vector<double> noise_levels;

int dimension;
Eigen::VectorXi lb;
Eigen::VectorXi ub;
double step_size;
double noise_level;

std::string test_problem;
nlohmann::json test_problem_config;
int total_num_configs;
int configs_done = 0;

bool track_experiments;
std::string experiment_name;
std::string experiment_id;
std::unique_ptr<MLflowLogger> mlflow_logger;

ProgressBar progress_bar;

auto main() -> int {
        parse_config();

        if (track_experiments) {
                mlflow_logger = std::make_unique<MLflowLogger>();
                experiment_id = mlflow_logger->get_or_create_experiment(experiment_name);
        }

        print_info();

        progress_bar = ProgressBar(total_num_configs, 50);

        for (const auto &dim : dimensions) {
                dimension = dim;
                for (const auto &noise_lvl : noise_levels) {
                        noise_level = noise_lvl;
                        parse_lb_ub();

                        run_all_configs(test_problem);
                }
        }

        progress_bar.finish();

        return 0;
}

auto single_run(const std::string &problem, const std::string &algo, const int num_particles, const bool use_random_location_update, const bool cap_velocity) {
        const int MAX_SIMULATIONS = config["MAX_SIMULATIONS"];

        const bool MEMORY = config["MEMORY"];

        const double W = config["W"];
        const double C1 = config["C1"];
        const double C2 = config["C2"];

        std::string time = std::to_string(std::time(nullptr));
        std::stringstream filename_stream("");
        filename_stream << problem << "_" << algo << "_memory_" << time << ".csv";

        double (*opti_func)(const Eigen::VectorXi &action_vector, bool noisy);
        if (problem == "inventory") {
                opti_func = inventory;
        } else if (problem == "styblinski-tang") {
                opti_func = styblinski_tang;
        } else if (problem == "eggholder") {
                opti_func = eggholder;
        } else if (problem == "michalewicz_ackley") {
                opti_func = michalewicz_ackley;
        } else if (problem == "michalewicz_sphere") {
                opti_func = michalewicz_sphere;
        } else if (problem == "f1_rosenbrock") {
                opti_func = rosenbrock;
        } else if (problem == "f2_six_hump_camel_back") {
                opti_func = six_hump_camel_back;
        } else if (problem == "f3_elliptic") {
                opti_func = elliptic;
        } else if (problem == "f4_ackley") {
                opti_func = ackley;
        } else {
                std::cerr << "Problem not found" << std::endl;
                exit(1);
        }

        if (algo == "pso") {
                PSO pso_instance = PSO(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                pso_instance.optimize();
                if (MEMORY) {
                        pso_instance.memory_to_csv(filename_stream.str());
                }
                return pso_instance.best_solutions();
        } else if (algo == "psoan") {
                PSOAN psoan_instance = PSOAN(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                psoan_instance.optimize();
                if (MEMORY) {
                        psoan_instance.memory_to_csv(filename_stream.str());
                }
                return psoan_instance.best_solutions();
        } else if (algo == "psogd") {
                PSOGD psogd_instance = PSOGD(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                psogd_instance.optimize();
                if (MEMORY) {
                        psogd_instance.memory_to_csv(filename_stream.str());
                }
                return psogd_instance.best_solutions();
        } else if (algo == "psoocba") {
                PSOOCBA psoocba_instance = PSOOCBA(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                psoocba_instance.optimize();
                if (MEMORY) {
                        psoocba_instance.memory_to_csv(filename_stream.str());
                }
                return psoocba_instance.best_solutions();
        } else if (algo == "psoocbaa") {
                PSOOCBAA psoocbaa_instance = PSOOCBAA(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                psoocbaa_instance.optimize();
                if (MEMORY) {
                        psoocbaa_instance.memory_to_csv(filename_stream.str());
                }
                return psoocbaa_instance.best_solutions();
        } else if (algo == "psola") {
                PSOLA psola_instance = PSOLA(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                psola_instance.optimize();
                if (MEMORY) {
                        psola_instance.memory_to_csv(filename_stream.str());
                }
                return psola_instance.best_solutions();
        } else if (algo == "lapso") {
                LAPSO lapso_instance = LAPSO(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                lapso_instance.optimize();
                if (MEMORY) {
                        lapso_instance.memory_to_csv(filename_stream.str());
                }
                return lapso_instance.best_solutions();
        } else if (algo == "psomab") {
                PSOMAB psomab_instance = PSOMAB(opti_func, lb, ub, dimension, MAX_SIMULATIONS, W, C1, C2, num_particles, use_random_location_update, cap_velocity);
                psomab_instance.optimize();
                if (MEMORY) {
                        psomab_instance.memory_to_csv(filename_stream.str());
                }
                return psomab_instance.best_solutions();
        } else if (algo == "psoern") {
                PSOERN psoern_instance = PSOERN(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, W, C1, C2, use_random_location_update, cap_velocity);
                psoern_instance.optimize();
                if (MEMORY) {
                        psoern_instance.memory_to_csv(filename_stream.str());
                }
                return psoern_instance.best_solutions();
        } else if (algo == "psoer") {
                PSOER psoer_instance = PSOER(num_particles, dimension, lb, ub, opti_func, MAX_SIMULATIONS, 20, W, C1, C2, use_random_location_update, cap_velocity);
                psoer_instance.optimize();
                if (MEMORY) {
                        psoer_instance.memory_to_csv(filename_stream.str());
                }
                return psoer_instance.best_solutions();
        } else {
                std::cerr << "Algorithm not found" << std::endl;
                exit(1);
        }
}

auto multiple_runs(const std::string &problem, const std::string &algo, const int num_particles, const bool use_random_location_update, const bool cap_velocity) {
        const int NUM_RUNS = config["NUM_RUNS"];
        std::vector<std::vector<solution>> all_solutions(NUM_RUNS);

        for (int i = 0; i < NUM_RUNS; ++i) {
                seed_generator(i);
                auto best_solutions = single_run(problem, algo, num_particles, use_random_location_update, cap_velocity);
                all_solutions[i] = best_solutions;
                double reward = 0.0;
                if (!best_solutions.empty()) {
                        reward = best_solutions.back().true_func_val;
                }
                //std::cout << "Run: " << i << " | best reward: " << reward << std::endl;
        }

        return all_solutions;
}

void run_config(const std::string &problem, const std::string &algo, const int num_particles, const bool use_random_location_update, const bool cap_velocity) {
        const int MAX_SIMULATIONS = config["MAX_SIMULATIONS"];
        const int NUM_RUNS = config["NUM_RUNS"];
        const bool to_csv_ = config["to_csv"];

        //std::cout << problem << " " << algo << std::endl;
        auto solutions = multiple_runs(problem, algo, num_particles, use_random_location_update, cap_velocity);

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

                mlflow_logger->log_param(run_id, "max_simulations", std::to_string(MAX_SIMULATIONS));
                mlflow_logger->log_param(run_id, "runs", std::to_string(NUM_RUNS));

                mlflow_logger->log_param(run_id, "num_particles", std::to_string(num_particles));
                mlflow_logger->log_param(run_id, "use_random_location_update", std::to_string(use_random_location_update));
                mlflow_logger->log_param(run_id, "cap_velocity", std::to_string(cap_velocity));

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

        configs_done++;

        progress_bar.update(configs_done);
}

void run_all_configs(const std::string &problem) {
        auto threads = std::vector<std::thread>();
        for (const auto &algo : algos) {
                for (const auto &num_particles : nums_particles) {
                        for (const auto &use_random_location_update : use_random_location_updates) {
                                for (const auto &cap_velocity : cap_velocities) {
                                        threads.emplace_back(run_config, problem, algo, num_particles, use_random_location_update, cap_velocity);
                                }
                        }
                }
        }

        for (auto &thread : threads) {
                thread.join();
        }
}

void parse_config() {
        config = nlohmann::json::parse(std::ifstream("config.json"));
        for (const auto &algo : config["algos"])
                algos.push_back(algo);
        for (const auto &num_particles : config["NUM_PARTICLES"])
                nums_particles.push_back(num_particles);
        for (const auto &use_random_location_update : config["USE_RANDOM_LOCATION_UPDATE"])
                use_random_location_updates.push_back(use_random_location_update);
        for (const auto &cap_velocity : config["CAP_VELOCITY"])
                cap_velocities.push_back(cap_velocity);

        test_problem = config["test_problem"];
        test_problem_config = config["problems"][test_problem];

        for (const auto &dim : test_problem_config["DIMENSIONS"])
                dimensions.push_back(dim);
        for (const auto &noise_lvl : test_problem_config["NOISE_LEVEL"])
                noise_levels.push_back(noise_lvl);
        step_size = test_problem_config["STEP_SIZE"];

        total_num_configs = static_cast<int>(dimensions.size() * noise_levels.size() * algos.size() * nums_particles.size() * use_random_location_updates.size() * cap_velocities.size());

        track_experiments = config["track_experiments"];
        experiment_name = config["experiment_name"];
}

void parse_lb_ub() {
        if (test_problem_config.contains("LOWER_BOUND_VECTOR") && test_problem_config.contains("UPPER_BOUND_VECTOR")) {
                std::vector<int> lb_input = test_problem_config["LOWER_BOUND_VECTOR"].get<std::vector<int>>();
                std::vector<int> ub_input = test_problem_config["UPPER_BOUND_VECTOR"].get<std::vector<int>>();

                if (lb_input.size() != dimension || ub_input.size() != dimension) {
                        std::cerr << "Dimension mismatch between dimension and lower/upper bound vector" << std::endl;
                        exit(1);
                }

                lb = Eigen::Map<Eigen::VectorXi>(lb_input.data(), dimension);
                ub = Eigen::Map<Eigen::VectorXi>(ub_input.data(), dimension);
        } else if (test_problem_config.contains("LOWER_BOUND") && test_problem_config.contains("UPPER_BOUND")) {
                lb = Eigen::VectorXi::Constant(dimension, test_problem_config["LOWER_BOUND"]);
                ub = Eigen::VectorXi::Constant(dimension, test_problem_config["UPPER_BOUND"]);
        } else {
                std::cerr << "No lower and upper bound given" << std::endl;
                exit(1);
        }
}

void print_info() {
        std::cout << "##########################################################################" << std::endl;
        std::cout << "Running problem: " << test_problem << std::endl;
        std::cout << "Number of dimension configurations:  " << dimensions.size() << std::endl;
        std::cout << "Number of noise level configurations:  " << noise_levels.size() << std::endl;
        std::cout << "\nTotal number of experiment configurations: " << dimensions.size() * noise_levels.size() << std::endl;
        std::cout << "##########################################################################" << std::endl;
        std::cout << "Algo size: " << algos.size() << std::endl;
        std::cout << "Number of num_particles configurations: " << nums_particles.size() << std::endl;
        std::cout << "Number of random_location_update configurations: " << use_random_location_updates.size() << std::endl;
        std::cout << "Number of cap_velocities configurations: " << cap_velocities.size() << std::endl;
        std::cout << "\nTotal number algorithm combinations: " << algos.size() * nums_particles.size() * use_random_location_updates.size() * cap_velocities.size() << std::endl;
        std::cout << "##########################################################################" << std::endl;
        std::cout << "Overall number of configurations: " << total_num_configs << std::endl;

        if (track_experiments) {
                std::cout << "\nResults will be tracked in experiment: " << experiment_name << std::endl;
        } else {
                std::cout << "\nResults will not be tracked" << std::endl;
        }
        std::cout << "##########################################################################" << std::endl;
}
