#include "PSOLA.h"
#include <cmath>
#include <iostream>
#include <iterator>

PSOLA::PSOLA(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update, bool cap_velocity) :pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update, cap_velocity) {
}

void PSOLA::sample_la() {
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
        double max = std::numeric_limits<double>::min();
        double min = std::numeric_limits<double>::max();

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                double mean_reward = pso.particles_[particle_index].mean_reward();
                if (mean_reward > max) {
                        max = mean_reward;
                }
                if (mean_reward < min) {
                        min = mean_reward;
                }
        }

        double add_term = 1e-10;
        for (int particle_index = 0; particle_index < pso.num_particle_; ++particle_index) {
                probabilities(particle_index) = (max - pso.particles_[particle_index].mean_reward() + add_term) / (max - min + add_term);
        }

        while (additional_simulations_done < additional_simulations_max && probabilities.maxCoeff() < threshold) {
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

                // update probabilities
                double helper_sum = 0;
                for (int particle_index = 0; particle_index < pso.num_particle_; ++particle_index) {
                        if (particle_index == sample_particle_index)
                                continue;
                        probabilities(particle_index) = std::max(0.0, probabilities(particle_index) - probability_penalty);
                        helper_sum += probabilities(particle_index);
                }
                probabilities(sample_particle_index) += helper_sum;
        }
        for (int particle_index = 0; particle_index < pso.num_particle_; ++particle_index) {
                if (pso.memory_active)
                        pso.save_particle_to_memory(pso.particles_[particle_index]);
        }
}

void PSOLA::update(){
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (pso.new_local_best(particle_index))
                        pso.best_individual_arms_[particle_index] = pso.particles_[particle_index];
                if (pso.new_global_best(particle_index))
                        pso.best_particle_index_ = particle_index;
        }
}

void PSOLA::optimize() {
        while (true) {
                sample_la();
                update();
                if (pso.budget_reached()) {
                        for (int particle_index = 0; particle_index < pso.num_particle_; ++particle_index) {
                                if (pso.memory_active)
                                        pso.save_particle_to_memory(pso.particles_[particle_index]);
                        }
                        return;
                }

                pso.update_positions();
        }
}

std::vector<solution> PSOLA::best_solutions() {
        return pso.best_solutions();
}
void PSOLA::memory_to_csv(const std::string &filename) {
        pso.memory_to_csv(filename);
}
