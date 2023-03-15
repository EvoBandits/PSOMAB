#ifndef PSOMAB_PACK_NAME_PSOMAB_PSO_H_
#define PSOMAB_PACK_NAME_PSOMAB_PSO_H_

#include "../../objects/arm/Arm.h"
#include "Eigen/Core"
#include <vector>
#include "../../objects/solution/Solution.h"
class PSO {
       public:
        PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false);
        PSO() = default;

        int num_particle() const;
        int dimension() const;
        Eigen::VectorXi x_min() const;
        Eigen::VectorXi x_max() const;
        std::function<double(Eigen::VectorXi, int)> opti_func() const;
        std::vector<Arm> particles() const;

        void optimize();

        void step(int num_pulls);
        int sum_num_pulls(std::vector<Arm> &arms) const;
        virtual std::vector<solution> &best_solutions();
        std::vector<Arm> &best_individual_arms();
        int max_simulation() const;
        int &best_particle_index();

       private:
        int num_particle_{};
        Eigen::VectorXi x_min_;                // D-dimensional Vector of the smallest possible
        // values a solution can have --> lower bounds
        Eigen::VectorXi x_max_;                // D-Dimensional Vector of the largest possible
        // values a solution can have --> upper bounds
        int dimension_;                        // D
        std::vector<Eigen::VectorXd> velocity_;// velocity
        std::function<double(Eigen::VectorXi, int)> opti_func_;
        std::vector<Arm> particles_;
        std::vector<Arm> best_individual_arms_;
        int best_particle_index_ = 0;
        void save_current_best_solution();
        std::vector<solution> best_solutions_;

        int max_simulation_;// max number of simulations

        std::vector<Arm> particles_history_;
        double c1 = 2.5;
        double c2 = 1;

        double w = 0.2;

        bool use_random_location_update_;
        Eigen::VectorXi update_location_random(Eigen::VectorXi proposed_solution);
        Eigen::VectorXi update_location_cap(Eigen::VectorXi proposed_solution);
};

#endif//PSOMAB_PACK_NAME_PSOMAB_PSO_H_
