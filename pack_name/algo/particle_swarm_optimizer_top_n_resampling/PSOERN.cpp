#include "PSOERN.h"

PSOERN::PSOERN(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update, bool cap_velocity) : pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update, cap_velocity) {
}

bool PSOERN::new_local_best(int particle_index) {
        bool better_reward_observed = pso.particles_[particle_index].mean_reward() < pso.best_individual_arms_[particle_index].mean_reward();
        bool no_local_best_yet = pso.best_individual_arms_[particle_index].reward() == 0;
        bool more_pulls = pso.particles_[particle_index].num_pulls() > pso.best_individual_arms_[particle_index].num_pulls();
        return better_reward_observed || no_local_best_yet || more_pulls;
}

bool PSOERN::new_global_best(int particle_index) {
        return pso.best_individual_arms_[particle_index].num_pulls() > pso.best_individual_arms_[pso.best_particle_index_].num_pulls();
}

std::vector<int> PSOERN::get_subset_indices() {
        Eigen::VectorXd current_mean_rewards(pso.num_particle_);
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                current_mean_rewards(particle_index) = pso.particles_[particle_index].mean_reward();
        }

        std::vector<int> sorted_indices = sort_indices(current_mean_rewards);
        sorted_indices.resize(top_n);

        return sorted_indices;
}

void PSOERN::update() {
        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                if (new_local_best(particle_index))
                        pso.best_individual_arms_[particle_index] = pso.particles_[particle_index];
                if (new_global_best(particle_index))
                        pso.best_particle_index_ = particle_index;
        }
}

void PSOERN::sample() {
        for (auto &particle : pso.particles_) {
                particle.pull(n_0);
                pso.update_simulation_budget(n_0);

                pso.save_history();
                if (pso.budget_reached())
                        return;
        }

        std::vector<int> subset_indices = get_subset_indices();
        for (auto &particle_index : subset_indices) {
                pso.particles_[particle_index].pull(additional_simulations / top_n);
                pso.update_simulation_budget(additional_simulations / top_n);

                pso.save_history();
                if (pso.budget_reached())
                        return;
        }
}

void PSOERN::optimize() {
        while (true) {
                sample();
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        if (pso.memory_active)
                                pso.save_particle_to_memory(pso.particles_[particle_index]);
                }
                update();

                if (pso.budget_reached())
                        return;

                pso.update_positions();
        }
}

std::vector<solution> PSOERN::best_solutions() {
        return pso.best_solutions();
}

void PSOERN::memory_to_csv(const std::string &filename) {
        pso.memory_to_csv(filename);
}