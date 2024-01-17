#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_TOP_N_RESAMPLING_PSOERN_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_TOP_N_RESAMPLING_PSOERN_H_

#include "../../../../util/RandomNumber.h"
#include "../../../../util/SortIndices.h"
#include "../../PSO.h"
#include <iostream>

/*
 * Based on:
 * Juan Rada-Vilela, Mengjie Zhang, and Mark Johnston. Resampling in particle swarm optimization.
 * Congress on Evolutionary Computation, pages 947–954. IEEE, 2013b
 */

class PSOERN {
       private:
        PSO pso;
        int top_n = 2;
        int n_0 = 5;
        int additional_simulations = 50;
        bool new_global_best(int particle_index);
        bool new_local_best(int particle_index);
        std::vector<int> get_subset_indices();
        void sample();
        void update();

       public:
        PSOERN(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, double w, double c1, double c2, bool use_random_location_update = false, bool cap_velocity = true);
        void optimize();
        std::vector<solution> best_solutions();
        void memory_to_csv(const std::string &filename);
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_TOP_N_RESAMPLING_PSOERN_H_
