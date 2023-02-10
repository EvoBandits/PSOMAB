#include "../pack_name/algo/PSOMAB/PSOMAB.h"
#include <iostream>

#include "problems/inventory.h"

int main() {

        Eigen::VectorXi x_lb(2);
        x_lb << 1, 1;
        Eigen::VectorXi x_ub(2);
        x_ub << 100, 100;

        int m = 10;

        PSOMAB instance = PSOMAB(inventory, 10000, m, 1, x_lb, x_ub, 2);
        instance.run();

        for (auto &best_solution : instance.getBest_solutions()) {
                std::cout << "budget:" << best_solution.obs_number << " | true:" << best_solution.true_func_val << "  mean:" << best_solution.mean_func_val << "  ";
                std::cout << "N: " << best_solution.N << "   ";
                for (int q : best_solution.x) {
                        std::cout << q << " ";
                }
                std::cout << std::endl;
        }

        return 0;
}
