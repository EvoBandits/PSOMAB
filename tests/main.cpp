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

#include <iostream>

#include "problem_parameter.h"
#include "problems/f1_rosenbrock/f1_rosenbrock.h"

// the following variables are defined as extern variables in problem_parameter.h
double step_size;
double noise_level;
int dimension;
Eigen::VectorXi lb;
Eigen::VectorXi ub;

void run_all_algorithms(int num_particles, int dimension, const Eigen::VectorXi &lb, const Eigen::VectorXi &ub,
                        int max_simulations, double W, double C1, double C2,
                        bool use_random_location_update, bool cap_velocity) {
        std::cout << "Running all algorithms on f1_rosenbrock problem" << std::endl;
        std::cout << "Dimension: " << dimension << ", Particles: " << num_particles << std::endl;
        std::cout << "Max Simulations: " << max_simulations << std::endl;
        std::cout << "========================================" << std::endl;

        // PSO
        {
                PSO pso_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                pso_instance.optimize();
                auto solutions = pso_instance.best_solutions();
                std::cout << "PSO:      Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOAN
        {
                PSOAN psoan_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                psoan_instance.optimize();
                auto solutions = psoan_instance.best_solutions();
                std::cout << "PSOAN:    Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOGD
        {
                PSOGD psogd_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                psogd_instance.optimize();
                auto solutions = psogd_instance.best_solutions();
                std::cout << "PSOGD:    Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOOCBA
        {
                PSOOCBA psoocba_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                psoocba_instance.optimize();
                auto solutions = psoocba_instance.best_solutions();
                std::cout << "PSOOCBA:  Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOOCBAA
        {
                PSOOCBAA psoocbaa_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                psoocbaa_instance.optimize();
                auto solutions = psoocbaa_instance.best_solutions();
                std::cout << "PSOOCBAA: Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOLA
        {
                PSOLA psola_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                psola_instance.optimize();
                auto solutions = psola_instance.best_solutions();
                std::cout << "PSOLA:    Best value = " << solutions.back().true_func_val << std::endl;
        }

        // LAPSO
        {
                LAPSO lapso_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                lapso_instance.optimize();
                auto solutions = lapso_instance.best_solutions();
                std::cout << "LAPSO:    Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOMAB
        {
                PSOMAB psomab_instance(rosenbrock, lb, ub, dimension, max_simulations, W, C1, C2, num_particles, use_random_location_update, cap_velocity);
                psomab_instance.optimize();
                auto solutions = psomab_instance.best_solutions();
                std::cout << "PSOMAB:   Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOERN
        {
                PSOERN psoern_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                psoern_instance.optimize();
                auto solutions = psoern_instance.best_solutions();
                std::cout << "PSOERN:   Best value = " << solutions.back().true_func_val << std::endl;
        }

        // PSOER
        {
                PSOER psoer_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, 20, W, C1, C2, use_random_location_update, cap_velocity);
                psoer_instance.optimize();
                auto solutions = psoer_instance.best_solutions();
                std::cout << "PSOER:    Best value = " << solutions.back().true_func_val << std::endl;
        }

        std::cout << "========================================" << std::endl;
        std::cout << "All algorithms completed!" << std::endl;
}

void run_repeated_comparison(int num_particles, int dimension, const Eigen::VectorXi &lb, const Eigen::VectorXi &ub,
                             int max_simulations, double W, double C1, double C2,
                             bool use_random_location_update, bool cap_velocity, int num_runs = 10) {
        std::cout << "Running repeated comparison (" << num_runs << " runs each)" << std::endl;
        std::cout << "Dimension: " << dimension << ", Particles: " << num_particles << std::endl;
        std::cout << "Max Simulations: " << max_simulations << std::endl;
        std::cout << "========================================" << std::endl;

        double pso_sum = 0.0;
        double lapso_sum = 0.0;
        double psomab_sum = 0.0;

        for (int run = 0; run < num_runs; ++run) {
                std::cout << "Run " << (run + 1) << "/" << num_runs << "..." << std::endl;

                // PSO
                {
                        PSO pso_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                        pso_instance.optimize();
                        auto solutions = pso_instance.best_solutions();
                        double best_val = solutions.back().true_func_val;
                        pso_sum += best_val;
                }

                // LAPSO
                {
                        LAPSO lapso_instance(num_particles, dimension, lb, ub, rosenbrock, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);
                        lapso_instance.optimize();
                        auto solutions = lapso_instance.best_solutions();
                        double best_val = solutions.back().true_func_val;
                        lapso_sum += best_val;
                }

                // PSOMAB
                {
                        PSOMAB psomab_instance(rosenbrock, lb, ub, dimension, max_simulations, W, C1, C2, num_particles, use_random_location_update, cap_velocity);
                        psomab_instance.optimize();
                        auto solutions = psomab_instance.best_solutions();
                        double best_val = solutions.back().true_func_val;
                        psomab_sum += best_val;
                }
        }

        double pso_avg = pso_sum / num_runs;
        double lapso_avg = lapso_sum / num_runs;
        double psomab_avg = psomab_sum / num_runs;

        std::cout << "========================================" << std::endl;
        std::cout << "Average Results over " << num_runs << " runs:" << std::endl;
        std::cout << "PSO:    " << pso_avg << std::endl;
        std::cout << "LAPSO:  " << lapso_avg << std::endl;
        std::cout << "PSOMAB: " << psomab_avg << std::endl;
        std::cout << "========================================" << std::endl;
}

auto main() -> int {
        // Problem setup
        step_size = 0.1;
        noise_level = 2.1;
        dimension = 10;
        const int lb_val = -5;
        const int ub_val = 10;
        lb = Eigen::VectorXi::Constant(dimension, lb_val);
        ub = Eigen::VectorXi::Constant(dimension, ub_val);

        // Algorithm parameters
        const int num_particles = 20;
        const int max_simulations = 100000;
        const double W = 0.7298;
        const double C1 = 1.49618;
        const double C2 = 1.49618;
        const bool use_random_location_update = false;
        const bool cap_velocity = true;

        // Uncomment the function you want to run:

        // Run all algorithms once
        run_all_algorithms(num_particles, dimension, lb, ub, max_simulations, W, C1, C2, use_random_location_update, cap_velocity);

        std::cout << std::endl;

        // Run PSO, LAPSO, and PSOMAB 10 times each and show averages
        run_repeated_comparison(num_particles, dimension, lb, ub, max_simulations, W, C1, C2, use_random_location_update, cap_velocity, 10);

        return 0;
}
