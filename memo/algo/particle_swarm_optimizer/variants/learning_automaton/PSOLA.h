#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_LEARNING_AUTOMATON_PSOLA_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_LEARNING_AUTOMATON_PSOLA_H_

#include "../../../../util/RandomNumber.h"
#include "../../../../util/SortIndices.h"
#include "../../PSO.h"
#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include <iostream>

/*
 * Based on:
 * JunQi Zhang, LinWei Xu, Jie Li, Qi Kang. Integrating Particle Swarm Optimization with Learning Automata to Solve Optimization Problems in Noisy Environment
 * 2014 IEEE International Conference on Systems, Man, and Cybernetics, 2014.
 */

class PSOLA {
       private:
        PSO pso;
        int n_0 = 5;
        int additional_simulations = 50;
        double probability_penalty = 0.001;
        double threshold = 0.7;

        void update();
        void sample_la();

       public:
        PSOLA(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, double w, double c1, double c2, bool use_random_location_update = false, bool cap_velocity = true);
        void optimize();
        std::vector<solution> best_solutions();
        void memory_to_csv(const std::string &filename);
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_LEARNING_AUTOMATON_PSOLA_H_
