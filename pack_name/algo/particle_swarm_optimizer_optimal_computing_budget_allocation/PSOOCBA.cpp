#include "PSOOCBA.h"

PSOOCBA::PSOOCBA(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update) : pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update) {
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

        int T = pso.num_particle_ * n_0;
        int T_max = T + iteration * 2;

        std::cout << "T_max: " << T_max << std::endl;
        std::cout << "T: " << T << std::endl;

        int delta = 0.1 * pso.num_particle_; // suggested choice for delta is a number bigger than 5 but smaller than 10% of the simulated designs

        int best_particle_index = -1;
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (best_particle_index == -1 || pso.particles_[particle_index].mean_reward() < pso.particles_[best_particle_index].mean_reward())
                        best_particle_index = particle_index;
        }

        while (T < T_max) {
                T += std::min(T_max - T, delta);
                std::cout << "T: " << T << std::endl;

                Eigen::VectorXi nprimes (pso.num_particle_);
                Eigen::VectorXd nprimes_weights (pso.num_particle_);
                double helper;

                // ToDo: check logic behind this (Theorem 1)
                double best_particle_mean_reward = pso.particles_[best_particle_index].mean_reward();
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        if (particle_index == best_particle_index)
                                continue;
                        double variance = pso.particles_[particle_index].variance();
                        double particle_mean_reward = pso.particles_[particle_index].mean_reward();
                        nprimes_weights(particle_index) = pow(variance / (best_particle_mean_reward - particle_mean_reward), 2);

                        std::cout << "val: " << nprimes_weights(particle_index)  << std::endl;

                        helper += pow(nprimes_weights(particle_index)/variance, 2);
                }
                nprimes_weights(best_particle_index) = pso.particles_[best_particle_index].variance() * pow(helper, 1/2);

                double nprimes_weights_sum = nprimes_weights.sum();

                // ToDo: Smart rounding
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        nprimes(particle_index) = std::round(delta * (nprimes_weights(particle_index) / nprimes_weights_sum));
                }

                std::cout << "nprimes sum: " << nprimes.sum() << std::endl;
                std::cout << "best: index: " << best_particle_index << std::endl;



                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        std::cout << "nprimes, pulls: " << nprimes(particle_index) << " | " << pso.particles_[particle_index].num_pulls() << " | "  << pso.particles_[particle_index].mean_reward() << " | "  << pso.particles_[particle_index].variance() << std::endl;
                        for (int i = 0; i < nprimes(particle_index); i++) {
                                pso.particles_[particle_index].pull();
                                pso.update_simulation_budget();

                                pso.save_history();
                                if (pso.budget_reached())
                                        return;
                        }
                }

                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        if (best_particle_index == -1 || pso.particles_[particle_index].mean_reward() < pso.particles_[best_particle_index].mean_reward())
                                best_particle_index = particle_index;
                }
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
                std::cout << "Budget: " << pso.simulations_used_ << std::endl;
                std::cout << "_______________________" << std::endl;
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
