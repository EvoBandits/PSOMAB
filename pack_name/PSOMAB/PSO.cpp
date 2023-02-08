#include "PSO.h"
#include "RandomNumber.h"
#include <iostream>

void PSO::step(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms, std::vector<std::vector<Arm>> arms_vec, std::vector<Arm> &current_particles, std::vector<Eigen::VectorXi> &velocity, int population_size, int dimension, Eigen::VectorXi vec_x_min, Eigen::VectorXi vec_x_max) {
        for (int k = 0; k < population_size; k++) {

                // with eigen for loop can bis discarded
                for (int g = 0; g < dimension; g++) {
                        // ToDo: ist runden hier richtig?
                        std::uniform_real_distribution<double> uniform_real_distribution_c1(0, c1);
                        std::uniform_real_distribution<double> uniform_real_distribution_c2(0, c2);
                        int b1 = round(uniform_real_distribution_c1(generator) * 1.0 * (best_individual_arms[k].get_action_vector()[g] - current_particles[k].get_action_vector()[g]));
                        int b2 = round(uniform_real_distribution_c2(generator) * 1.0 * (arms_vec[best_global_particle_index][best_global_index].get_action_vector()[g] - current_particles[k].get_action_vector()[g]));

                        velocity[k][g] = round(w * velocity[k][g] + b1 + b2);

                        int new_value = current_particles[k].get_action_vector()[g] + velocity[k][g];

                        // ToDo: ist das das richtige vorgehen bei werten außerhalb der range?
                        if (new_value > vec_x_max[g] || new_value < vec_x_min[g]) {
                                std::uniform_int_distribution<int> uniform_int_distribution(vec_x_min[g], vec_x_max[g]);
                                new_value = uniform_int_distribution(generator);
                        }
                        current_particles[k].set_action_vector_element(g, new_value);
                }
        }
}
PSO::PSO() = default;
