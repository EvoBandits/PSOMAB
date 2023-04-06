#include "PSOOCBA.h"
#include <cmath>
#include <iostream>

PSOOCBA::PSOOCBA(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update) :pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update) {
}

int PSOOCBA::find_best_particle_index() {
        int best_particle_index = -1;

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (best_particle_index == -1 || pso.particles_[particle_index].mean_reward() < pso.particles_[best_particle_index].mean_reward())
                        best_particle_index = particle_index;
        }

        return best_particle_index;
}

void PSOOCBA::sample_ocba(int iteration) {
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
        int additional_simulations_max = additional_simulations_done + iteration * 2;

        int delta = std::max((int) 0.1 * pso.num_particle_, 1); // suggested choice for delta is a number bigger than 5 but smaller than 10% of the simulated designs

        int best_particle_index = find_best_particle_index();

        double add_term = 1e-10;

        while (additional_simulations_done < additional_simulations_max) {
                additional_simulations_done += std::min(additional_simulations_max - additional_simulations_done, delta);

                Eigen::VectorXi addition_simulations = Eigen::VectorXi::Zero(pso.num_particle_);
                Eigen::VectorXd weights = Eigen::VectorXd::Zero(pso.num_particle_);
                double helper_weight_best_particle = 0;
                double best_particle_mean_reward = pso.particles_[best_particle_index].mean_reward();

                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        if (particle_index == best_particle_index) {
                                continue;
                        }

                        double variance = pso.particles_[particle_index].variance();
                        double particle_mean_reward = pso.particles_[particle_index].mean_reward();

                        weights(particle_index) = pow((variance + add_term) / (best_particle_mean_reward - particle_mean_reward + add_term), 2);

                        helper_weight_best_particle += pow((weights(particle_index) + add_term) / (variance + add_term), 2);
                }
                weights(best_particle_index) = pso.particles_[best_particle_index].variance() * pow(helper_weight_best_particle, 0.5);

                double weights_sum = weights.sum();
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        weights(particle_index) = delta * (weights(particle_index) / weights_sum);
                }

                addition_simulations = smart_rounding(weights, delta);

                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        for (int i = 0; i < addition_simulations(particle_index); i++) {
                                pso.particles_[particle_index].pull();
                                pso.update_simulation_budget();

                                pso.save_history();
                                if (pso.budget_reached())
                                        return;
                        }
                }

                best_particle_index = find_best_particle_index();
        }
}

void PSOOCBA::update(){
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (pso.new_local_best(particle_index))
                        pso.best_individual_arms_[particle_index] = pso.particles_[particle_index];
                if (pso.new_global_best(particle_index))
                        pso.best_particle_index_ = particle_index;
        }
}

void PSOOCBA::optimize() {
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

std::vector<solution> PSOOCBA::best_solutions() {
        return pso.best_solutions();
}
