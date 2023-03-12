#include "PSO.h"
#include "../../util/RandomNumber.h"
#include <iostream>
#include <utility>


Eigen::VectorXi PSO::update_location_random(Eigen::VectorXi proposed_solution){
        for (int j = 0; j < dimension_; j++) {
                if (proposed_solution[j] > x_max_[j] || proposed_solution[j] < x_min_[j]) {
                        std::uniform_int_distribution<int> uniform_int_distribution(x_min_[j], x_max_[j]);
                        proposed_solution[j] = uniform_int_distribution(generator);
                }
        }
        return proposed_solution;
}

Eigen::VectorXi PSO::update_location_cap(Eigen::VectorXi proposed_solution){
        for (int j = 0; j < dimension_; j++) {
                if (proposed_solution[j] < x_min_[j]) proposed_solution[j] = x_min_[j];
                else if (proposed_solution[j] > x_max_[j]) proposed_solution[j] =  x_max_[j];
        }
        return proposed_solution;
}

void PSO::step(int num_pulls) {
        for (int particle_index = 0; particle_index < num_particle_; particle_index++) {
                // Update velocity
                Eigen::VectorXd cognitive_direction = (best_individual_arms_[particle_index].get_action_vector() - particles_[particle_index].get_action_vector()).cast<double>();
                Eigen::VectorXd social_direction = (best_individual_arms_[best_particle_index_].get_action_vector() - particles_[particle_index].get_action_vector()).cast<double>();

                std::uniform_real_distribution<double> distribution_0_1(0, 1);

                Eigen::VectorXd old_velocity = w * velocity_[particle_index].cast<double>();
                Eigen::VectorXd social_component = (distribution_0_1(generator) * c2) * social_direction;
                Eigen::VectorXd cognitive_component = (distribution_0_1(generator) * c1) * cognitive_direction;

                Eigen::VectorXd new_velocity = old_velocity + cognitive_component + social_component;

                // ToDo: check if cap_velocity is necessary/usefully
                //cap_velocity(new_velocity);

                velocity_[particle_index] = new_velocity;

                // update position
                Eigen::VectorXi proposed_position = particles_[particle_index].get_action_vector() + velocity_[particle_index].cast<int>();
                //Eigen::VectorXi new_position = update_location_random(proposed_position);
                Eigen::VectorXi new_position = update_location_cap(proposed_position);

                Arm new_arm = Arm(opti_func_, new_position);
                particles_[particle_index] = new_arm;

                // in case of PSOMAB: pull arm 0 times, as arm gets pulled in PSOMAB
                // in case of PSO: pull arm 1 or more times (k times)
                for (int k = 0; k < num_pulls; k++) {
                        particles_[particle_index].pull();
                }

                // save history, used for output
                // ToDo: replace particles_history_ with some kind of counter-variable as it is currently just used to track the number of simulations (<-> PSOMAB)
                particles_history_.push_back(particles_[particle_index]);

                // fix problem when best_individual_arm has never been pulled
                // ToDo: was das?
                if (best_individual_arms_[particle_index].reward() == 0) {
                        best_individual_arms_[particle_index] = particles_[particle_index];
                }

                // Check if new personal best
                // ToDo: not useful when using PSOMAB, as we already get the best arm in PSOMAB
                if (particles_[particle_index].mean_reward() < best_individual_arms_[particle_index].mean_reward()) {
                        best_individual_arms_[particle_index] = particles_[particle_index];
                }

                // check if new global best
                if (best_individual_arms_[particle_index].mean_reward() < best_individual_arms_[best_particle_index_].mean_reward()) {
                        best_particle_index_ = particle_index;
                }
        }
}

void PSO::optimize() {
        while (true) {
                step(1);
                if (sum_num_pulls(particles_history_) % 100 == 0) {
                        save_current_best_solution();
                }
                if (sum_num_pulls(particles_history_) > max_simulation_) {
                        break;
                }
        }
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

PSO::PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation) : num_particle_{num_particle}, dimension_{dimension}, x_min_{std::move(x_min)}, x_max_{std::move(x_max)}, opti_func_{std::move(opti_func)}, max_simulation_{max_simulation} {
        std::vector<Eigen::VectorXi> init_solutions;
        for (int i = 0; i < num_particle_; i++) {
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
        particles_history_ = particles_;
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
std::function<double(Eigen::VectorXi, int)> PSO::opti_func() const {
        return opti_func_;
}
std::vector<Arm> PSO::particles() const {
        return particles_;
}
int PSO::sum_num_pulls(std::vector<Arm> &arms) const {
        int sum = 0;
        for (auto &arm : arms) {
                sum += arm.num_pulls();
        }
        return sum;
}
void PSO::save_current_best_solution() {
        best_solutions_.emplace_back(sum_num_pulls(particles_history_), best_individual_arms_[best_particle_index_].get_action_vector(), best_individual_arms_[best_particle_index_].num_pulls(), best_individual_arms_[best_particle_index_].mean_reward(), best_individual_arms_[best_particle_index_].true_value());
}
std::vector<solution> &PSO::best_solutions() {
        return best_solutions_;
}
int PSO::max_simulation() const {
        return max_simulation_;
}
std::vector<Arm> &PSO::best_individual_arms() {
        return best_individual_arms_;
}
int &PSO::best_particle_index() {
        return best_particle_index_;
}
