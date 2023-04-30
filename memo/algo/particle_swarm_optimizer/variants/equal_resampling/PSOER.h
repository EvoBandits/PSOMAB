#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_EQUAL_RESAMPLING_PSOER_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_EQUAL_RESAMPLING_PSOER_H_

#include "../../../../util/RandomNumber.h"
#include "../../PSO.h"

class PSOER {
       private:
        PSO pso;
        void sample_and_update(int particle_index);
        int num_pulls_;

       public:
        PSOER(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, int num_sample, bool use_random_location_update=false, bool cap_velocity=true);
        void optimize();
        std::vector<solution> best_solutions();
        void memory_to_csv(const std::string& filename);
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_EQUAL_RESAMPLING_PSOER_H_
