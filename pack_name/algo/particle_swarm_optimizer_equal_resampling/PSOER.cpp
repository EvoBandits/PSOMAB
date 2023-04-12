#include "PSOER.h"

void PSOER::sample_and_update(int particle_index) {
        pso.particles_[particle_index].pull(num_pulls_);
        pso.update_simulation_budget(num_pulls_);

        if (pso.new_local_best(particle_index))
                pso.best_individual_arms_[particle_index] = pso.particles_[particle_index];
        if (pso.new_global_best(particle_index))
                pso.best_particle_index_ = particle_index;
}

void PSOER::optimize() {
        while (true) {
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        sample_and_update(particle_index);

                        pso.save_history();
                        if (pso.memory_active)
                                pso.save_particle_to_memory(particle_index);
                        if (pso.budget_reached())
                                return;
                }

                pso.update_positions();
        }
}

PSOER::PSOER(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, int num_sample, bool use_random_location_update, bool cap_velocity) : pso(num_particle, dimension, x_min, x_max, std::move(opti_func), max_simulation, use_random_location_update, cap_velocity), num_pulls_{num_sample} {
}

std::vector<solution> PSOER::best_solutions() {
        return pso.best_solutions();
}
void PSOER::memory_to_csv(const std::string &filename) {
        pso.memory_to_csv(filename);
}
