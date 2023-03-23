#ifndef PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_AVERAGE_NEIGHBORHOOD_PSOAN_H_
#define PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_AVERAGE_NEIGHBORHOOD_PSOAN_H_

#include "../particle_swarm_optimizer/PSO.h"

class PSOAN {
       private:
        PSO pso;


       public:
        PSOAN(std::function<double(Eigen::VectorXi, int)> func, int max_sim, int pop_s, const Eigen::VectorXi &s_lb, const Eigen::VectorXi &s_ub, int D, bool use_random_location_update=false);
        void optimize();
        std::vector<solution> best_solutions();
};

#endif//PSOMAB_PACK_NAME_ALGO_PARTICLE_SWARM_OPTIMIZER_AVERAGE_NEIGHBORHOOD_PSOAN_H_
