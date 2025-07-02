#include "PSOAN.h"

#include <utility>

void PSOAN::calculate_distance_matrix(Eigen::MatrixXd &distance_matrix) {
        for (int particle_index_1 = 0; particle_index_1 < pso.num_particle_; particle_index_1++) {
                for (int particle_index_2 = particle_index_1; particle_index_2 < pso.num_particle_; particle_index_2++) {
                        if (particle_index_1 == particle_index_2) {
                                distance_matrix(particle_index_1, particle_index_2) = 0;
                        } else {
                                double distance = (pso.particles_[particle_index_1].get_action_vector() - pso.particles_[particle_index_2].get_action_vector()).norm();
                                distance_matrix(particle_index_1, particle_index_2) = distance;
                                distance_matrix(particle_index_2, particle_index_1) = distance;
                        }
                }
        }
}

void PSOAN::calculate_averaged_best_individual_arms(std::vector<Eigen::VectorXd> &averaged_best_individual_arms) {
        Eigen::MatrixXd distance_matrix(pso.num_particle_, pso.num_particle_);
        calculate_distance_matrix(distance_matrix);

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                Eigen::VectorXd averaged_best_individual_arm = Eigen::VectorXd::Zero(pso.dimension_);
                std::vector<int> indices_sorted = sort_indices(distance_matrix.row(particle_index));
                for (int i = 0; i < neighborhood_size; i++) {
                        averaged_best_individual_arm += (pso.best_individual_arms_[indices_sorted[i]].get_action_vector()).cast<double>();
                }
                averaged_best_individual_arm /= neighborhood_size;
                averaged_best_individual_arms.push_back(averaged_best_individual_arm);
        }
}

void PSOAN::update_positions() {
        std::vector<Eigen::VectorXd> averaged_best_individual_arms;
        calculate_averaged_best_individual_arms(averaged_best_individual_arms);

        for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                Eigen::VectorXd current_position = pso.particles_[particle_index].get_action_vector().cast<double>();
                Eigen::VectorXd local_best_position = pso.best_individual_arms_[particle_index].get_action_vector().cast<double>();
                Eigen::VectorXd avg_neighborhood_best_position = averaged_best_individual_arms[particle_index];

                Eigen::VectorXd cognitive_direction = local_best_position - current_position;
                Eigen::VectorXd social_direction = avg_neighborhood_best_position - current_position;

                Eigen::VectorXd old_velocity = pso.w_ * pso.velocity_[particle_index].cast<double>();
                Eigen::VectorXd social_component = (random_uniform_double(0, 1) * pso.c2_) * social_direction;
                Eigen::VectorXd cognitive_component = (random_uniform_double(0, 1) * pso.c1_) * cognitive_direction;

                Eigen::VectorXd new_velocity = old_velocity + cognitive_component + social_component;
                if (pso.cap_velocity_) pso.cap_velocity(new_velocity);

                pso.velocity_[particle_index] = new_velocity;
                Eigen::VectorXi proposed_position = pso.particles_[particle_index].get_action_vector() + pso.velocity_[particle_index].cast<int>();

                Eigen::VectorXi new_position;
                if (pso.use_random_location_update_)
                        new_position = pso.update_location_random(proposed_position);
                else
                        new_position = pso.update_location_cap(proposed_position);

                Arm new_arm = Arm(pso.opti_func_, new_position);
                pso.particles_[particle_index] = new_arm;
        }
}

PSOAN::PSOAN(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, double w, double c1, double c2, bool use_random_location_update, bool cap_velocity) : pso(num_particle, dimension, std::move(x_min), std::move(x_max), std::move(opti_func), max_simulation, w, c1, c2, use_random_location_update, cap_velocity) {
}

void PSOAN::optimize() {
        while (true) {
                for (int particle_index = 0; particle_index < pso.num_particle_; particle_index++) {
                        pso.sample_and_update(particle_index);

                        pso.save_history();
                        if (pso.memory_active)
                                pso.save_particle_to_memory(pso.particles_[particle_index]);
                        if (pso.budget_reached())
                                return;
                }

                update_positions();
        }
}

std::vector<solution> PSOAN::best_solutions() {
        return pso.best_solutions();
}
void PSOAN::memory_to_csv(const std::string &filename) {
        pso.memory_to_csv(filename);
}
