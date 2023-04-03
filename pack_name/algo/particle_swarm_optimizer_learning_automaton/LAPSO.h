#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_LAPSO_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_LAPSO_H_

#include <iostream>
#include "Eigen/Core"
#include "../particle_swarm_optimizer/PSO.h"
#include "../../util/SortIndices.h"
#include "../../util/RandomNumber.h"
#include "Eigen/src/Core/Matrix.h"

/*
 * Based on:
 * Hui Pan, Ling Wang, and Bo Liu. Particle swarm optimization for function optimization in noisy environment.
 * Applied mathematics and Computation, 181(2):908–919, 2006.
 */

class LAPSO {
       private:
        PSO pso;
        int n_0 = 5; // suitable choice for n0 is between 5 and 20 (Law and Kelton [1991]; Bechhofer et al. [1995])
        int additional_simulations = 50;
        double probability_penalty = 0.001;
        double threshold = 0.7;
        Arm global_best_arm = Arm(pso.opti_func_, Eigen::VectorXi(pso.num_particle_));

       public:
        LAPSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false);
        void optimize();
        std::vector<solution> best_solutions();
        void update();
        void sample_la();
        void update_positions();
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_LAPSO_H_
