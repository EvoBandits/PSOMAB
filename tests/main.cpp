#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include <iostream>

#include "problems/inventory.h"

int main() {

        Eigen::VectorXi x_lb(2);
        x_lb << 1, 1;
        Eigen::VectorXi x_ub(2);
        x_ub << 100, 100;

        //PSO pso_instance = PSO(10, 2, x_lb, x_ub, inventory);
        //pso_instance.optimize();

        PSOMAB psomab_instance = PSOMAB(inventory, 10000, 10, 1, x_lb, x_ub, 2);
        psomab_instance.optimize();

        for (auto &best_solution : psomab_instance.getBest_solutions()) {
                std::cout << "budget:" << best_solution.obs_number << " | true:" << best_solution.true_func_val << "  mean:" << best_solution.mean_func_val << "  ";
                std::cout << "N: " << best_solution.N << "   ";
                for (int q : best_solution.x) {
                        std::cout << q << " ";
                }
                std::cout << std::endl;
        }

        return 0;
}
