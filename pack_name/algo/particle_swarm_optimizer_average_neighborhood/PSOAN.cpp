#include "PSOAN.h"
#include "../../util/RandomNumber.h"
void PSOAN::optimize() {
        while (true) {
                pso.update_positions();

                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        pso.sample_and_update(particle_index);

                        pso.save_history();
                        if (pso.budget_reached())
                                return;
                }
        }
}
PSOAN::PSOAN(std::function<double(Eigen::VectorXi, int)> func, int max_sim, int pop_s, const Eigen::VectorXi &s_lb, const Eigen::VectorXi &s_ub, int D, bool use_random_location_update) : pso(pop_s, D, s_lb, s_ub, std::move(func), max_sim, use_random_location_update) {
}

std::vector<solution> PSOAN::best_solutions() {
        return pso.best_solutions();
}
