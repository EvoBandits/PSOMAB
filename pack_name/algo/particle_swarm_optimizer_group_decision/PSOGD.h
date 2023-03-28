#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_GROUP_DECISION_PSOGD_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_GROUP_DECISION_PSOGD_H_

#include "../particle_swarm_optimizer/PSO.h"
#include "../../util/RandomNumber.h"
#include "../../util/SortIndices.h"
#include "limits.h"
#include <iostream>

    class PSOGD {
       private:
        PSO pso;
        int decision_layer_size = pso.num_particle_*0.3;
        double alpha = 2;

        void update_positions();
        Eigen::VectorXd calculate_search_center();

       public:
        PSOGD(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false);
        void optimize();
        std::vector<solution> best_solutions();
    };

#endif PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_GROUP_DECISION_PSOGD_H_
