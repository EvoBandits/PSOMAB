#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer_average_neighborhood/PSOAN.h"
#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include <iostream>

#include "problems/inventory/inventory.h"
#include "problems/tp1/tp1.h"
#include "problems/ackley/ackley.h"

int main() {
        std::string algo = "pso";

        if(algo == "pso"){
                PSO pso_instance = PSO(10, inventory_dim, inventory_lb, inventory_ub, inventory, 10000);
                pso_instance.optimize();

                for (auto &best_solution : pso_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psoan"){
                PSOAN psoan_instance = PSOAN(10, inventory_dim, inventory_lb, inventory_ub, inventory, 10000);
                psoan_instance.optimize();

                for (auto &best_solution : psoan_instance.best_solutions()) {
                        best_solution.print();
                }
        } else if (algo == "psomab"){
                PSOMAB psomab_instance = PSOMAB(ackley, 10000, 10, ackley_lb, ackley_ub, ackley_dim, false);
                psomab_instance.optimize();

                for (auto &best_solution : psomab_instance.best_solutions()) {
                        best_solution.print();
                }
        }

        return 0;
}
