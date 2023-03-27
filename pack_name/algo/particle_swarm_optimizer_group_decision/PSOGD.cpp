#include "PSOGD.h"

void PSOGD::calculate_search_center(Eigen::VectorXd matrix) {
}

void PSOGD::update_positions() {
        Eigen::VectorXd search_center;
        calculate_search_center(search_center);

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                Eigen::VectorXd current_position = pso.particles_[particle_index].get_action_vector().cast<double>();
                Eigen::VectorXd local_best_position = pso.best_individual_arms_[particle_index].get_action_vector().cast<double>();

                Eigen::VectorXd cognitive_direction = local_best_position - current_position;
                Eigen::VectorXd social_direction = search_center- current_position;

                Eigen::VectorXd old_velocity = pso.w * pso.velocity_[particle_index].cast<double>();
                Eigen::VectorXd social_component = (random_uniform_double(0,1) * pso.c2) * social_direction;
                Eigen::VectorXd cognitive_component = (random_uniform_double(0,1) * pso.c1) * cognitive_direction;

                Eigen::VectorXd new_velocity = old_velocity + cognitive_component + social_component;

                // ToDo: check if cap_velocity is necessary/usefully
                //cap_velocity(new_velocity);

                pso.velocity_[particle_index] = new_velocity;
                Eigen::VectorXi proposed_position = pso.particles_[particle_index].get_action_vector() + pso.velocity_[particle_index].cast<int>();

                Eigen::VectorXi new_position;
                if(pso.use_random_location_update_)
                new_position = pso.update_location_random(proposed_position);
                else
                new_position = pso.update_location_cap(proposed_position);

                Arm new_arm = Arm(pso.opti_func_, new_position);
                pso.particles_[particle_index] = new_arm;
        }
}

PSOGD::PSOGD(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update) : pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update) {
}

void PSOGD::optimize() {
        while (true) {
                update_positions();

                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                pso.sample_and_update(particle_index);

                pso.save_history();
                if (pso.budget_reached())
                        return;
                }
        }
}

std::vector<solution> PSOGD::best_solutions() {
        return pso.best_solutions();
}
