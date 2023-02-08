#ifndef PSOMAB_PACK_NAME_PSOMAB_PSO_H_
#define PSOMAB_PACK_NAME_PSOMAB_PSO_H_

#include "Eigen/Core"
#include <vector>
#include "Arm.h"
class PSO {
       public:
        PSO();

        void step(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms, std::vector<std::vector<Arm>> arms_vec, std::vector<Arm> &current_particles, std::vector<Eigen::VectorXi> &velocity, int population_size, int dimension, Eigen::VectorXi vec_x_min, Eigen::VectorXi vec_x_max);

       private:

        double c1 = 2.5;
        double c2 = 1;
        double w = 0.2;

};

#endif//PSOMAB_PACK_NAME_PSOMAB_PSO_H_
