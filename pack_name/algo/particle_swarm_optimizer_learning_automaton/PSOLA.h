#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_PSOLA_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_PSOLA_H_

#include <iostream>
#include "Eigen/Core"
#include "../particle_swarm_optimizer/PSO.h"
#include "../../util/SortIndices.h"
#include "../../util/RandomNumber.h"
#include "Eigen/src/Core/Matrix.h"

/*
 * Based on:
 * JunQi Zhang, LinWei Xu, Jie Li, Qi Kang. Integrating Particle Swarm Optimization with Learning Automata to Solve Optimization Problems in Noisy Environment
 * 2014 IEEE International Conference on Systems, Man, and Cybernetics, 2014.
 */

class PSOLA {
       private:
        PSO pso;
        int n_0 = 5; // suitable choice for n0 is between 5 and 20 (Law and Kelton [1991]; Bechhofer et al. [1995])
        int additional_simulations = 50;
        double probability_penalty = 0.001;
        double threshold = 0.7;
        Arm global_best_arm = Arm(pso.opti_func_, Eigen::VectorXi(pso.num_particle_));

       public:
        PSOLA(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false);
        void optimize();
        std::vector<solution> best_solutions();
        void update();
        void sample_la();
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_PSOLA_H_
