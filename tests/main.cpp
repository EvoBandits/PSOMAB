#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer_average_neighborhood/PSOAN.h"
#include "../pack_name/algo/particle_swarm_optimizer_group_decision/PSOGD.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation/PSOOCBA.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation_adjusted/PSOOCBAA.h"
#include "../pack_name/algo/particle_swarm_optimizer_learning_automaton/LAPSO.h"
#include "../pack_name/algo/particle_swarm_optimizer_learning_automaton/PSOLA.h"
#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include <iostream>

#include "problems/inventory/inventory.h"
#include "problems/tp1/tp1.h"
#include "problems/ackley/ackley.h"

int main() {
        std::string algo = "lapso";
        int max_simulation = 10000;
        int num_particle = 50;
        int dimension = inventory_dim;
        Eigen::VectorXi lb = inventory_lb;
        Eigen::VectorXi ub = inventory_ub;
        std::function<double(Eigen::VectorXi, int)> opti_func = inventory;


        if(algo == "pso"){
                std::cout << "PSO:" << std::endl;
                PSO pso_instance = PSO(num_particle, dimension, lb, ub, opti_func, max_simulation);
                pso_instance.optimize();

                for (auto &best_solution : pso_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoan"){
                std::cout << "PSO-AN:" << std::endl;
                PSOAN psoan_instance = PSOAN(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psoan_instance.optimize();

                for (auto &best_solution : psoan_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psogd"){
                std::cout << "PSO-GD:" << std::endl;
                PSOGD psogd_instance = PSOGD(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psogd_instance.optimize();

                for (auto &best_solution : psogd_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoocba"){
                std::cout << "PSO-OCBA:" << std::endl;
                PSOOCBA psoocba_instance = PSOOCBA(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psoocba_instance.optimize();

                for (auto &best_solution : psoocba_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoocbaa"){
                std::cout << "PSO-OCBA adjusted:" << std::endl;
                PSOOCBAA psoocbaa_instance = PSOOCBAA(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psoocbaa_instance.optimize();

                for (auto &best_solution : psoocbaa_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psola"){
                std::cout << "PSOLA" << std::endl;
                PSOLA psola_instance = PSOLA(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psola_instance.optimize();

                for (auto &best_solution : psola_instance.best_solutions()) {
                        best_solution.print();
                }
        }
        else if (algo == "lapso"){
                std::cout << "LAPSO" << std::endl;
                LAPSO lapso_instance = LAPSO(num_particle, dimension, lb, ub, opti_func, max_simulation);
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

        /*

        double mean_reward = 0;
        int runs = 500;

        std::cout << std::fixed;
        std::cout << std::setprecision(3);

        for (int i = 0; i < runs; i++) {
                LAPSO lapso_instance = LAPSO(num_particle, dimension, lb, ub, opti_func, max_simulation);
                lapso_instance.optimize();
                std::cout << "Run: " << i  << " | best reward: " << lapso_instance.best_solutions().back().true_func_val << std::endl;
                mean_reward += lapso_instance.best_solutions().back().true_func_val;
        }

        std::cout << "Mean reward: " << mean_reward / runs << std::endl;
        */

        return 0;
}
