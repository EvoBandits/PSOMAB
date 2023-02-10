#ifndef PSOMAB_PACK_NAME_PSOMAB_PSO_H_
#define PSOMAB_PACK_NAME_PSOMAB_PSO_H_

#include "Eigen/Core"
#include <vector>
#include "../../Arm.h"
class PSO {
       public:
        PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi)> opti_func);
        PSO() = default;

        void step(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms, std::vector<std::vector<Arm>> arms_vec);

        int num_particle() const;
        int dimension() const;
        Eigen::VectorXi x_min() const;
        Eigen::VectorXi x_max() const;
        std::function<double(Eigen::VectorXi)> opti_func() const;
        std::vector<Arm> particles() const;

       private:
        int num_particle_{};
        Eigen::VectorXi x_min_;// D-dimensional Vector of the smallest possible
                                  // values a solution can have --> lower bounds
        Eigen::VectorXi x_max_;// D-Dimensional Vector of the largest possible
                                  // values a solution can have --> upper bounds
        int dimension_;// D
        std::vector<Eigen::VectorXi> velocity_;// velocity
        std::function<double(Eigen::VectorXi)> opti_func_;
        std::vector<Arm> particles_;

        double c1 = 2.5;
        double c2 = 1;
        double w = 0.2;

};

#endif//PSOMAB_PACK_NAME_PSOMAB_PSO_H_
