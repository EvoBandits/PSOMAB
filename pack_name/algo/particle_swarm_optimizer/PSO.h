#ifndef PSOMAB_PACK_NAME_PSOMAB_PSO_H_
#define PSOMAB_PACK_NAME_PSOMAB_PSO_H_

#include "../../objects/arm/Arm.h"
#include "Eigen/Core"
#include <vector>
#include "../../objects/solution/Solution.h"

/*
 * Based on:
 * James Kennedy and Russell Eberhart. Particle swarm optimization.
 * In Proceedings of ICNN’95-international conference on neural networks, volume 4, pages 1942–1948. IEEE, 1995
 */

class PSO {
       public:
        PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update=false, bool cap_velocity=true);
        void optimize();
        std::vector<solution> &best_solutions();

        void update_positions();
        void sample_and_update(int particle_index);
        void update_simulation_budget(int number_of_new_simulations=1);
        void save_history();
        void save_current_best_solution();

        bool budget_reached() const;
        bool new_local_best(int particle_index);
        bool new_global_best(int particle_index);

        void calculate_max_velocity();
        void cap_velocity(Eigen::VectorXd &velocity);
        Eigen::VectorXi update_location_random(Eigen::VectorXi proposed_solution);
        Eigen::VectorXi update_location_cap(Eigen::VectorXi proposed_solution);

        int num_particle() const;
        int dimension() const;
        std::vector<Arm> particles() const;
        std::vector<Arm> &best_individual_arms();
        int simulations_used() const;
        int &best_particle_index();


        //parameter
        int num_particle_;
        double c1 = 2.5;
        double c2 = 1;
        double w = 0.2;
        bool use_random_location_update_;
        bool cap_velocity_ = true;
        int max_simulations_;

        std::function<double(Eigen::VectorXi, int)> opti_func_;
        int dimension_;
        Eigen::VectorXi lb;
        Eigen::VectorXi ub;


        int simulations_used_ = 0;
        std::vector<Arm> particles_;
        Eigen::VectorXd max_velocity_;
        std::vector<Eigen::VectorXd> velocity_;
        std::vector<Arm> best_individual_arms_;
        int best_particle_index_ = 0;
        std::vector<solution> best_solutions_;

};

#endif//PSOMAB_PACK_NAME_PSOMAB_PSO_H_
