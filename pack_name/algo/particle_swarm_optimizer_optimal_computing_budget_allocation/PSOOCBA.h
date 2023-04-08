#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_OPTIMAL_COMPUTING_BUDGET_ALLOCATION_PSOOCBA_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_OPTIMAL_COMPUTING_BUDGET_ALLOCATION_PSOOCBA_H_

#include "Eigen/Core"
#include "../particle_swarm_optimizer/PSO.h"
#include "../../util/SortIndices.h"
#include "../../util/SmartRounding.h"
#include <iostream>

/*
 * Based on:
 * Hui Pan, Ling Wang, and Bo Liu. Particle swarm optimization for function optimization in noisy environment.
 * Applied mathematics and Computation, 181(2):908–919, 2006.
 */

class PSOOCBA {
       private:
        PSO pso;
        int n_0 = 5; // suitable choice for n0 is between 5 and 20 (Law and Kelton [1991]; Bechhofer et al. [1995])

       public:
        PSOOCBA(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false, bool cap_velocity=true);
        void optimize();
        std::vector<solution> best_solutions();
        void update();
        void sample_ocba(int iteration);
        int find_best_particle_index();
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_OPTIMAL_COMPUTING_BUDGET_ALLOCATION_PSOOCBA_H_
