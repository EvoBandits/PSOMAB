#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_LAPSO_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_LAPSO_H_

#include "../../../../util/RandomNumber.h"
#include "../../../../util/SortIndices.h"
#include "../../PSO.h"
#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include <iostream>

/*
 * Based on:
 * JunQi Zhang, LinWei Xu, Ji Ma, and MengChu Zhou. A learning automata-based particle swarm optimization algorithm for noisy environment.
 * In 2015 IEEE Congress on Evolutionary Computation (CEC), pages 141–147. IEEE, 2015
 */

class LAPSO {
       private:
        PSO pso;
        int n_0 = 1;
        int additional_simulations = 250;
        double probability_penalty = 0.001;
        double threshold = 0.9;
        int subset_capacity = 3;
        Arm global_best_arm = Arm(pso.opti_func_, Eigen::VectorXi(pso.num_particle_));

        void update();
        void sample_la();
        void update_positions();
        std::vector<int> get_subset_indices();

       public:
        LAPSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false, bool cap_velocity=true);
        void optimize();
        std::vector<solution> best_solutions();
        void memory_to_csv(const std::string& filename);
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER__LEARNING_AUTOMATON_LAPSO_H_
