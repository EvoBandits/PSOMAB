#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer_average_neighborhood/PSOAN.h"
#include "../pack_name/algo/particle_swarm_optimizer_group_decision/PSOGD.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation/PSOOCBA.h"
#include "../pack_name/algo/particle_swarm_optimizer_optimal_computing_budget_allocation_adjusted/PSOOCBAA.h"
#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include <iostream>

#include "problems/inventory/inventory.h"
#include "problems/tp1/tp1.h"
#include "problems/ackley/ackley.h"

int main() {
        std::string algo = "psoocbaa";
        int max_simulation = 10000;
        int num_particle = 10;
        int dimension = inventory_dim;
        Eigen::VectorXi lb = inventory_lb;
        Eigen::VectorXi ub = inventory_ub;
        std::function<double(Eigen::VectorXi, int)> opti_func = inventory;


        if(algo == "pso"){
                PSO pso_instance = PSO(num_particle, dimension, lb, ub, opti_func, max_simulation);
                pso_instance.optimize();

                for (auto &best_solution : pso_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoan"){
                PSOAN psoan_instance = PSOAN(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psoan_instance.optimize();

                for (auto &best_solution : psoan_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psogd"){
                PSOGD psogd_instance = PSOGD(50, dimension, lb, ub, opti_func, max_simulation);
                psogd_instance.optimize();

                for (auto &best_solution : psogd_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoocba"){
                PSOOCBA psoocba_instance = PSOOCBA(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psoocba_instance.optimize();

                for (auto &best_solution : psoocba_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoocbaa"){
                PSOOCBAA psoocbaa_instance = PSOOCBAA(num_particle, dimension, lb, ub, opti_func, max_simulation);
                psoocbaa_instance.optimize();

                for (auto &best_solution : psoocbaa_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psomab"){
                PSOMAB psomab_instance = PSOMAB(opti_func, max_simulation, num_particle, lb, ub, dimension, false);
                psomab_instance.optimize();

                for (auto &best_solution : psomab_instance.best_solutions()) {
                        best_solution.print();
                }
        }

        return 0;
}
