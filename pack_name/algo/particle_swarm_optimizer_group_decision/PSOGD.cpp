#include "PSOGD.h"

Eigen::VectorXd PSOGD::calculate_search_center() {
        Eigen::VectorXd rewards_all(pso.num_particle_);
        for (int i = 0; i < pso.num_particle_; i++) {
                rewards_all(i) = pso.particles_[i].mean_reward();
        }

        std::vector<int> indices_gd_maker_layer = sort_indices(rewards_all);
        indices_gd_maker_layer.resize(decision_layer_size);

        Eigen::VectorXd rewards_gd_maker_layer = rewards_all(indices_gd_maker_layer);

        double min = rewards_gd_maker_layer.minCoeff();
        double max = rewards_gd_maker_layer.maxCoeff();

        Eigen::VectorXd decision_weights = exp(-(rewards_gd_maker_layer.array() - min) / (max - min));

        double sum_decision_weights = decision_weights.sum();
        decision_weights /= sum_decision_weights;

        Eigen::VectorXd search_center = Eigen::VectorXd::Zero(pso.dimension_);
        for (int i = 0; i < decision_layer_size; i++) {
                search_center+= decision_weights(i) * pso.particles_[indices_gd_maker_layer[i]].get_action_vector().cast<double>();
        }

        return search_center;
}

void PSOGD::update_positions() {
        Eigen::VectorXd search_center = calculate_search_center();

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                Eigen::VectorXd current_position = pso.particles_[particle_index].get_action_vector().cast<double>();

                Eigen::VectorXd social_direction = search_center - current_position;

                Eigen::VectorXd old_velocity = random_uniform_double(0,1) * pso.w * pso.velocity_[particle_index].cast<double>();
                Eigen::VectorXd social_component = random_uniform_double(0,1) * alpha* social_direction;

                Eigen::VectorXd new_velocity = old_velocity + social_component;

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
        pso.w = 0.4;
        alpha = 3.2;
}

void PSOGD::optimize() {
        while (true) {
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        pso.sample_and_update(particle_index);

                        pso.save_history();
                        if (pso.budget_reached()) {
                                return;
                        }
                }
                update_positions();
        }
}

std::vector<solution> PSOGD::best_solutions() {
        return pso.best_solutions();
}
