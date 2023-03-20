#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include <iostream>

#include "problems/inventory.h"

int main() {

        Eigen::VectorXi x_lb(2);
        x_lb << 1, 1;
        Eigen::VectorXi x_ub(2);
        x_ub << 100, 100;

        bool pso = false;

        if(pso){
                PSO pso_instance = PSO(10, 2, x_lb, x_ub, inventory, 10000);
                pso_instance.optimize();

                for (auto &best_solution : pso_instance.best_solutions()) {
                        best_solution.print();
                }
        } else {
                //ToDo: Seed setzen
                PSOMAB psomab_instance = PSOMAB(inventory, 10000, 10, 1, x_lb, x_ub, 2, false);
                psomab_instance.optimize();

                for (auto &best_solution : psomab_instance.best_solutions()) {
                        best_solution.print();
                }
        }


        return 0;
}
