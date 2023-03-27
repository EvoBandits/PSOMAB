#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_AVERAGE_NEIGHBORHOOD_PSOAN_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_AVERAGE_NEIGHBORHOOD_PSOAN_H_

#include "../particle_swarm_optimizer/PSO.h"
#include "../../util/RandomNumber.h"
#include "../../util/SortIndices.h"
#include <iostream>

    class PSOAN {
       private:
        PSO pso;
        int neighborhood_size = pso.num_particle_*0.3;
        void update_positions();
        void calculate_averaged_best_individual_arms(std::vector<Eigen::VectorXd> &averaged_best_individual_arms);
        void calculate_distance_matrix(Eigen::MatrixXd &distance_matrix);


       public:
        PSOAN(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false);
        void optimize();
        std::vector<solution> best_solutions();
};

#endif PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_AVERAGE_NEIGHBORHOOD_PSOAN_H_
