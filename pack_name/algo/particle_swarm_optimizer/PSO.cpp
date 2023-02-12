#include "PSO.h"
#include "../../util/RandomNumber.h"
#include <iostream>
#include <utility>

void PSO::step(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms, std::vector<std::vector<Arm>> arms_vec) {
        for (int k = 0; k < num_particle_; k++) {

                // with eigen for loop can bis discarded
                for (int g = 0; g < dimension_; g++) {
                        // ToDo: ist runden hier richtig?
                        std::uniform_real_distribution<double> uniform_real_distribution_c1(0, c1);
                        std::uniform_real_distribution<double> uniform_real_distribution_c2(0, c2);
                        int b1 = round(uniform_real_distribution_c1(generator) * 1.0 * (best_individual_arms[k].get_action_vector()[g] - particles_[k].get_action_vector()[g]));
                        int b2 = round(uniform_real_distribution_c2(generator) * 1.0 * (arms_vec[best_global_particle_index][best_global_index].get_action_vector()[g] - particles_[k].get_action_vector()[g]));

                        velocity_[k][g] = round(w * velocity_[k][g] + b1 + b2);

                        int new_value = particles_[k].get_action_vector()[g] + velocity_[k][g];

                        // ToDo: ist das das richtige vorgehen bei werten außerhalb der range?
                        if (new_value > x_max_[g] || new_value < x_min_[g]) {
                                std::uniform_int_distribution<int> uniform_int_distribution(x_min_[g], x_max_[g]);
                                new_value = uniform_int_distribution(generator);
                        }
                        particles_[k].set_action_vector_element(g, new_value);
                }
        }
}

void PSO::step() {
        for (int i = 0; i < num_particle_; i++) {
                // Update velocity
                Eigen::VectorXd vec1 = (best_individual_arms_[i].get_action_vector() - particles_[i].get_action_vector()).cast<double>();
                Eigen::VectorXd vec2 = (best_individual_arms_[best_global_arm_index_].get_action_vector() - particles_[i].get_action_vector()).cast<double>();
                std::uniform_real_distribution<double> distribution_0_1(0, 1);
                Eigen::VectorXd new_velocity = w * velocity_[i].cast<double>() + (distribution_0_1(generator) * c1) * vec1 + (distribution_0_1(generator) * c2) * vec2;
                //cap_velocity(new_velocity);
                velocity_[i] = new_velocity;

                // Update location
                Eigen::VectorXi new_vector = particles_[i].get_action_vector() + velocity_[i].cast<int>();

                // check if new location is in range
                for (int j = 0; j < dimension_; j++) {
                        if (new_vector[j] > x_max_[j]) {
                                new_vector[j] = x_max_[j];
                        } else if (new_vector[j] < x_min_[j]) {
                                new_vector[j] = x_min_[j];
                        }
                }

                Arm new_arm = Arm(opti_func_, new_vector);
                particles_[i] = new_arm;
                particles_[i].pull();

                // fix problem when best_individual_arm has never been pulled
                if (best_individual_arms_[i].reward() == 0) {
                        best_individual_arms_[i] = particles_[i];
                }

                // Check if new personal best
                if(particles_[i].mean_reward() < best_individual_arms_[i].mean_reward()) {
                        best_individual_arms_[i] = particles_[i];
                }
                // check if new global best
                if(best_individual_arms_[i].mean_reward() < best_individual_arms_[best_global_arm_index_].mean_reward()) {
                        best_global_arm_index_ = i;
                }
        }
}

void PSO::optimize() {
        for (int i = 0; i < 1000; i++) {
                step();
        }
        std::cout << "Best global arm: " << best_individual_arms_[best_global_arm_index_].get_action_vector().transpose() << std::endl;
        std::cout << "Best global reward: " << best_individual_arms_[best_global_arm_index_].mean_reward() << std::endl;
}

Eigen::VectorXi generate_unique_solution(std::vector<Eigen::VectorXi> &solutions, Eigen::VectorXi x_lb, Eigen::VectorXi x_ub, int dimension) {
        Eigen::VectorXi v(dimension);
        // generate random solutions as long as they are not unique
        while (true) {
                for (int j = 0; j < dimension; j++) {
                        std::uniform_int_distribution<int> uniform_dist(x_lb(j), x_ub(j));
                        v(j) = uniform_dist(generator);
                }
                if (std::find(solutions.begin(), solutions.end(), v) == solutions.end()) {
                        break;
                }
        }
        return v;
}

PSO::PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi)> opti_func) : num_particle_{num_particle}, dimension_{dimension}, x_min_{std::move(x_min)}, x_max_{std::move(x_max)}, opti_func_{std::move(opti_func)} {
        std::vector<Eigen::VectorXi> init_solutions;
        for(int i = 0; i < num_particle_; i++) {
                velocity_.emplace_back(Eigen::VectorXd::Zero(dimension_));
                // initialize vector of solutions
                Eigen::VectorXi v(dimension_);
                // generate random solutions as long as they are not unique
                v = generate_unique_solution(init_solutions, x_min_, x_max_, dimension_);
                init_solutions.push_back(v);//required to check wheather all elements are unique
                // add arm to "arms", i.e. where all arms are stored
                Arm new_arm(opti_func_, v);// 0 = cost info, last element (0) is actually not necessary
                particles_.push_back(new_arm);
        }
        // initialize best individual arms
        // ToDo: in eigenes init
        best_individual_arms_ = particles_;
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
std::function<double(Eigen::VectorXi)> PSO::opti_func() const {
        return opti_func_;
}
std::vector<Arm> PSO::particles() const {
        return particles_;
}
