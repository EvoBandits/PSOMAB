#include "../pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "../pack_name/algo/particle_swarm_optimizer/PSO.h"
#include <iostream>

#include "problems/inventory/inventory.h"
#include "problems/tp1/tp1.h"
#include "problems/ackley/ackley.h"

int main() {
        bool pso = false;

        if(pso){
                PSO pso_instance = PSO(10, inventory_dim, inventory_lb, inventory_ub, inventory, 10000);
                pso_instance.optimize();

                for (auto &best_solution : pso_instance.best_solutions()) {
                        best_solution.print();
                }
        } else {
                //ToDo: Seed setzen
                PSOMAB psomab_instance = PSOMAB(ackley, 10000, 10, 1, ackley_lb, ackley_ub, ackley_dim, false);
                psomab_instance.optimize();

                for (auto &best_solution : psomab_instance.best_solutions()) {
                        best_solution.print();
                }
        }



        return 0;
}
