#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer_average_neighborhood/PSOAN.h"
#include "../pack_name/algo/particle_swarm_optimizer_group_decision/PSOGD.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation/PSOOCBA.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation_adjusted/PSOOCBAA.h"
#include "../pack_name/algo/particle_swarm_optimizer_learning_automaton/LAPSO.h"
#include "../pack_name/algo/particle_swarm_optimizer_learning_automaton/PSOLA.h"
#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include "../pack_name/algo/particle_swarm_optimizer_equal_resampling/PSOER.h"
#include <iostream>

#include "problems/inventory/inventory.h"
#include "problems/tp1/tp1.h"
#include "problems/ackley/ackley.h"

int main() {
        std::string algo = "psoer";
        int max_simulation = 10000;
        int num_particle = 50;
        bool use_random_location_update = false;
        bool cap_velocity = true;

        int dimension = inventory_dim;
        Eigen::VectorXi lb = inventory_lb;
        Eigen::VectorXi ub = inventory_ub;
        std::function<double(Eigen::VectorXi, int)> opti_func = inventory;


        if(algo == "pso"){
                PSO pso_instance = PSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                pso_instance.optimize();

                for (auto &best_solution : pso_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoan"){
                PSOAN psoan_instance = PSOAN(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoan_instance.optimize();

                for (auto &best_solution : psoan_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psogd"){
                PSOGD psogd_instance = PSOGD(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psogd_instance.optimize();

                for (auto &best_solution : psogd_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoocba"){
                PSOOCBA psoocba_instance = PSOOCBA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoocba_instance.optimize();

                for (auto &best_solution : psoocba_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoocbaa"){
                PSOOCBAA psoocbaa_instance = PSOOCBAA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psoocbaa_instance.optimize();

                for (auto &best_solution : psoocbaa_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psola"){
                std::cout << "PSOLA" << std::endl;
                PSOLA psola_instance = PSOLA(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                psola_instance.optimize();

                for (auto &best_solution : psola_instance.best_solutions()) {
                        best_solution.print();
                }
        }
        else if (algo == "lapso"){
                std::cout << "LAPSO" << std::endl;
                LAPSO lapso_instance = LAPSO(num_particle, dimension, lb, ub, opti_func, max_simulation, use_random_location_update, cap_velocity);
                lapso_instance.optimize();

                for (auto &best_solution : lapso_instance.best_solutions()) {
                        best_solution.print();
                }
        }
        else if (algo == "psomab"){
                std::cout << "PSOMAB:" << std::endl;
                PSOMAB psomab_instance = PSOMAB(opti_func, max_simulation, num_particle, lb, ub, dimension, false);
                psomab_instance.optimize();

                for (auto &best_solution : psomab_instance.best_solutions()) {
                        best_solution.print();
                }
        }
        else if (algo == "psoer"){
                std::cout << "PSOER:" << std::endl;
                PSOER psoer_instance = PSOER(num_particle, dimension, lb, ub, opti_func, max_simulation, 20, use_random_location_update, cap_velocity);
                psoer_instance.optimize();

                for (auto &best_solution : psoer_instance.best_solutions()) {
                        best_solution.print();
                }
        }

        /*

        double mean_reward = 0;
        int runs = 500;

        std::cout << std::fixed;
        std::cout << std::setprecision(3);

        for (int i = 0; i < runs; i++) {
                PSOMAB psomab_instance = PSOMAB(opti_func, max_simulation, num_particle, lb, ub, dimension, use_random_location_update, cap_velocity);
                psomab_instance.optimize();
                std::cout << "Run: " << i  << " | best reward: " << psomab_instance.best_solutions().back().true_func_val << std::endl;
                mean_reward += psomab_instance.best_solutions().back().true_func_val;
        }

        std::cout << "Mean reward: " << mean_reward / runs << std::endl;
        */

        return 0;
}
