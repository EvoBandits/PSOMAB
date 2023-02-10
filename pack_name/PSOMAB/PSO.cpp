#include "PSO.h"
#include "RandomNumber.h"
#include <iostream>
#include <utility>

void PSO::step(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms, std::vector<std::vector<Arm>> arms_vec, std::vector<Arm> &current_particles) {
        for (int k = 0; k < num_particle_; k++) {

                // with eigen for loop can bis discarded
                for (int g = 0; g < dimension_; g++) {
                        // ToDo: ist runden hier richtig?
                        std::uniform_real_distribution<double> uniform_real_distribution_c1(0, c1);
                        std::uniform_real_distribution<double> uniform_real_distribution_c2(0, c2);
                        int b1 = round(uniform_real_distribution_c1(generator) * 1.0 * (best_individual_arms[k].get_action_vector()[g] - current_particles[k].get_action_vector()[g]));
                        int b2 = round(uniform_real_distribution_c2(generator) * 1.0 * (arms_vec[best_global_particle_index][best_global_index].get_action_vector()[g] - current_particles[k].get_action_vector()[g]));

                        velocity_[k][g] = round(w * velocity_[k][g] + b1 + b2);

                        int new_value = current_particles[k].get_action_vector()[g] + velocity_[k][g];

                        // ToDo: ist das das richtige vorgehen bei werten außerhalb der range?
                        if (new_value > x_max_[g] || new_value < x_min_[g]) {
                                std::uniform_int_distribution<int> uniform_int_distribution(x_min_[g], x_max_[g]);
                                new_value = uniform_int_distribution(generator);
                        }
                        current_particles[k].set_action_vector_element(g, new_value);
                }
        }
}
PSO::PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max) : num_particle_{num_particle}, dimension_{dimension}, x_min_{std::move(x_min)}, x_max_{std::move(x_max)} {
        for(int i = 0; i < num_particle_; i++) {
                velocity_.emplace_back(Eigen::VectorXi::Zero(dimension_));
        }
}
int PSO::num_particle() const {
        return num_particle_;
}
int PSO::dimension() const {
        return dimension_;
}
Eigen::VectorXi PSO::x_min() const {
        return x_min_;
}
Eigen::VectorXi PSO::x_max() const {
        return x_max_;
}
