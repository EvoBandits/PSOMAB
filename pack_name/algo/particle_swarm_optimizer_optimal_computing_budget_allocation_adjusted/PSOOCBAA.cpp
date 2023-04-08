#include "PSOOCBAA.h"
#include <cmath>
#include <iostream>

PSOOCBAA::PSOOCBAA(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update, bool cap_velocity) :pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update, cap_velocity) {
}

int PSOOCBAA::get_num_pulls(int index) {
        if (index >= pso.num_particle_)
                return pso.best_individual_arms()[index - pso.num_particle_].num_pulls();
        else
                return pso.particles_[index].num_pulls();
}

double PSOOCBAA::get_mean_reward(int index) {
        if (index >= pso.num_particle_)
                return pso.best_individual_arms()[index - pso.num_particle_].mean_reward();
        else
                return pso.particles_[index].mean_reward();
}

double PSOOCBAA::get_variance(int index) {
        if (index >= pso.num_particle_)
                return pso.best_individual_arms()[index - pso.num_particle_].variance();
        else
                return pso.particles_[index].variance();
}

int PSOOCBAA::find_best_particle_index(int num_participating_particles) {
        int best_particle_index = -1;

        for (int particle_index = 0; particle_index < num_participating_particles; particle_index++) {
                if (best_particle_index == -1 || get_mean_reward(particle_index) < get_mean_reward(best_particle_index))
                        best_particle_index = particle_index;
        }

        return best_particle_index;
}

void PSOOCBAA::sample_ocba(int iteration) {
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

        int additional_simulations_used = pso.num_particle_ * n_0;
        int additional_simulations_max = additional_simulations_used + iteration * 2;


        // include the pbest of each particle in the ocba procedure
        int num_participating_particles = pso.num_particle_;
        if (iteration > 0){
                num_participating_particles += pso.num_particle_;
        }

        int delta = std::max((int) 0.1 * num_participating_particles, 1); // suggested choice for delta is a number bigger than 5 but smaller than 10% of the simulated designs

        int best_particle_index = find_best_particle_index(num_participating_particles);

        double add_term = 1e-10;

        while (additional_simulations_used < additional_simulations_max) {
                additional_simulations_used += std::min(additional_simulations_max - additional_simulations_used, delta);

                Eigen::VectorXi addition_simulations = Eigen::VectorXi::Zero(num_participating_particles);
                Eigen::VectorXd weights = Eigen::VectorXd::Zero(num_participating_particles);
                double helper_weight_best_particle = 0;
                double best_particle_mean_reward = get_mean_reward(best_particle_index);

                for (int particle_index = 0; particle_index < num_participating_particles; particle_index++) {
                        if (particle_index == best_particle_index)
                                continue;

                        double variance = get_variance(particle_index);
                        double particle_mean_reward = get_mean_reward(particle_index);

                        weights(particle_index) = pow((variance + add_term)/ (best_particle_mean_reward - particle_mean_reward + add_term), 2);

                        helper_weight_best_particle += pow((weights(particle_index) + add_term) / (variance + add_term), 2);
                }
                weights(best_particle_index) = get_variance(best_particle_index) * pow(helper_weight_best_particle, 0.5);

                double weights_sum = weights.sum();
                for (int particle_index = 0; particle_index < num_participating_particles; particle_index++) {
                        weights(particle_index) = delta * (weights(particle_index) / weights_sum);
                }

                addition_simulations = smart_rounding(weights, delta);

                for (int particle_index = 0; particle_index < num_participating_particles; particle_index++) {
                        for (int i = 0; i < addition_simulations(particle_index); i++) {
                                if (particle_index >= pso.num_particle_)
                                        pso.best_individual_arms_[particle_index - pso.num_particle_].pull();
                                else
                                        pso.particles_[particle_index].pull();
                                pso.update_simulation_budget();

                                pso.save_history();
                                if (pso.budget_reached())
                                        return;
                        }
                }

                best_particle_index = find_best_particle_index(num_participating_particles);
        }
}

void PSOOCBAA::update(){
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (pso.new_local_best(particle_index))
                        pso.best_individual_arms_[particle_index] = pso.particles_[particle_index];
                if (pso.new_global_best(particle_index))
                        pso.best_particle_index_ = particle_index;
        }
}

void PSOOCBAA::optimize() {
        int iteration = 0;

        while (true) {
                sample_ocba(iteration);
                if (pso.budget_reached())
                        return;

                update();
                pso.update_positions();
                iteration++;
        }
}

std::vector<solution> PSOOCBAA::best_solutions() {
        return pso.best_solutions();
}
