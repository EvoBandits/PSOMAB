#include "LAPSO.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <vector>

LAPSO::LAPSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update, bool cap_velocity) : pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update, cap_velocity) {
}

std::vector<int> LAPSO::get_subset_indices() {
        Eigen::VectorXd current_mean_rewards(pso.num_particle_);
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                current_mean_rewards(particle_index) = pso.particles_[particle_index].mean_reward();
        }

        std::vector<int> sorted_indices = sort_indices(current_mean_rewards);
        sorted_indices.resize(subset_capacity);

        return sorted_indices;
}

void LAPSO::sample_la() {
        // collect n0 samples for each Xi
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                for (int i = 0; i < n_0; i++) {
                        pso.particles_[particle_index].pull();
                        pso.update_simulation_budget();

                        pso.save_history();
                        if (pso.budget_reached())
                                return;
                }
        }

        int additional_simulations_done = pso.num_particle_ * n_0;
        int additional_simulations_max = additional_simulations_done + additional_simulations;

        Eigen::VectorXd probabilities = Eigen::VectorXd::Zero(pso.num_particle_);

        for (int particle_index = 0; particle_index < pso.num_particle_; ++particle_index) {
                probabilities(particle_index) = 1.0/pso.num_particle_;
        }

        std::vector<int> subset_indices = get_subset_indices();

        // adapted because of mistake in paper
        while (additional_simulations_done < additional_simulations_max && probabilities(subset_indices).sum() < threshold) {
                additional_simulations_done += 1;

                // distribution
                std::discrete_distribution<int> distribution(probabilities.data(), probabilities.data() + probabilities.size());
                int sample_particle_index = distribution(generator);

                // pull
                pso.particles_[sample_particle_index].pull();
                pso.update_simulation_budget();

                pso.save_history();
                if (pso.budget_reached())
                        return;

                // determine subset (best subset_capacity-particles)
               subset_indices = get_subset_indices();

                // update probabilities
                double helper_sum = 0;
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        if (std::find(subset_indices.begin(), subset_indices.end(), particle_index) != subset_indices.end())
                                continue;
                        probabilities(particle_index) = std::max(0.0, probabilities(particle_index) - probability_penalty);
                        helper_sum += probabilities(particle_index);
                }
                probabilities(subset_indices) = ((1 - helper_sum) / subset_capacity) * Eigen::VectorXd::Ones(subset_capacity);
        }
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (pso.memory_active)
                        pso.save_particle_to_memory(pso.particles_[particle_index]);
        }
}

void LAPSO::update(){
        int current_best_particle_index = -1;
        double current_best_reward = std::numeric_limits<double>::max();

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (pso.new_local_best(particle_index))
                        pso.best_individual_arms_[particle_index] = pso.particles_[particle_index];
                if (pso.new_global_best(particle_index))
                        pso.best_particle_index_ = particle_index;
                if (pso.particles_[particle_index].mean_reward() < current_best_reward) {
                        current_best_reward = pso.particles_[particle_index].mean_reward();
                        current_best_particle_index = particle_index;
                }
        }

        global_best_arm = pso.particles_[current_best_particle_index];
}

void LAPSO::update_positions() {
        Eigen::VectorXd global_best_position = global_best_arm.get_action_vector().cast<double>();

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                Eigen::VectorXd current_position = pso.particles_[particle_index].get_action_vector().cast<double>();
                Eigen::VectorXd local_best_position = pso.best_individual_arms_[particle_index].get_action_vector().cast<double>();

                Eigen::VectorXd cognitive_direction = local_best_position - current_position;
                Eigen::VectorXd social_direction = global_best_position - current_position;

                Eigen::VectorXd old_velocity = pso.w * pso.velocity_[particle_index].cast<double>();
                Eigen::VectorXd social_component = (random_uniform_double(0,1) * pso.c2) * social_direction;
                Eigen::VectorXd cognitive_component = (random_uniform_double(0,1) * pso.c1) * cognitive_direction;

                Eigen::VectorXd new_velocity = old_velocity + cognitive_component + social_component;
                if (pso.cap_velocity_) pso.cap_velocity(new_velocity);

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

void LAPSO::optimize() {
        int iteration = 0;

        while (true) {
                sample_la();
                update();

                if (pso.budget_reached()) {
                        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                                if (pso.memory_active)
                                        pso.save_particle_to_memory(pso.particles_[particle_index]);
                        }
                        return;
                }
                update_positions();
                iteration++;
        }
}

std::vector<solution> LAPSO::best_solutions() {
        return pso.best_solutions();
}
void LAPSO::memory_to_csv(const std::string &filename) {
        pso.memory_to_csv(filename);
}
