#ifndef PSOMAB_PACK_NAME_PSOMAB_PSO_H_
#define PSOMAB_PACK_NAME_PSOMAB_PSO_H_

#include "../../objects/arm/Arm.h"
#include "Eigen/Core"
#include <vector>
#include "../../objects/solution/Solution.h"

class PSO {
       public:
        PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false);
        int num_particle() const;
        int dimension() const;
        std::vector<Arm> particles() const;
        void optimize();
        void update_positions();
        bool budget_reached() const;
        virtual std::vector<solution> &best_solutions();
        std::vector<Arm> &best_individual_arms();
        int simulations_used() const;
        void update_simulation_budget(int number_of_new_simulations=1);
        void save_current_best_solution();
        void save_history();
        int &best_particle_index();

       private:
        // hyperparameters
        int dimension_;
        std::function<double(Eigen::VectorXi, int)> opti_func_;
        Eigen::VectorXi x_min_; // lower bounds
        Eigen::VectorXi x_max_; // upper bounds
        int num_particle_;
        double c1 = 2.5;
        double c2 = 1;
        double w = 0.2;
        int max_simulations_;
        int simulations_used_ = 0;
        bool use_random_location_update_;

        std::vector<Arm> particles_;
        std::vector<Eigen::VectorXd> velocity_;
        std::vector<Arm> best_individual_arms_;
        int best_particle_index_ = 0;
        std::vector<solution> best_solutions_;

        Eigen::VectorXi update_location_random(Eigen::VectorXi proposed_solution);
        Eigen::VectorXi update_location_cap(Eigen::VectorXi proposed_solution);
        bool new_local_best(int particle_index);
        bool new_global_best(int particle_index);
        void sample_and_update(int particle_index);
};

#endif//PSOMAB_PACK_NAME_PSOMAB_PSO_H_
