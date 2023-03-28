#include "PSO.h"
#include "../../util/RandomNumber.h"
#include <iostream>
#include <utility>


Eigen::VectorXi PSO::update_location_random(Eigen::VectorXi proposed_solution){
        for (int j = 0; j < dimension_; j++) {
                if (proposed_solution[j] > x_max_[j] || proposed_solution[j] < x_min_[j]) {
                        proposed_solution[j] = random_uniform_int(x_min_[j], x_max_[j]);
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

void PSO::update_positions() {
        Eigen::VectorXd global_best_position = best_individual_arms_[best_particle_index_].get_action_vector().cast<double>();

        for (int particle_index = 0; particle_index < num_particle_; particle_index++) {
                Eigen::VectorXd current_position = particles_[particle_index].get_action_vector().cast<double>();
                Eigen::VectorXd local_best_position = best_individual_arms_[particle_index].get_action_vector().cast<double>();

                Eigen::VectorXd cognitive_direction = local_best_position - current_position;
                Eigen::VectorXd social_direction = global_best_position - current_position;

                Eigen::VectorXd old_velocity = w * velocity_[particle_index].cast<double>();
                Eigen::VectorXd social_component = (random_uniform_double(0,1) * c2) * social_direction;
                Eigen::VectorXd cognitive_component = (random_uniform_double(0,1) * c1) * cognitive_direction;

                Eigen::VectorXd new_velocity = old_velocity + cognitive_component + social_component;

                // ToDo: check if cap_velocity is necessary/usefully
                //cap_velocity(new_velocity);

                velocity_[particle_index] = new_velocity;
                Eigen::VectorXi proposed_position = particles_[particle_index].get_action_vector() + velocity_[particle_index].cast<int>();

                Eigen::VectorXi new_position;
                if(use_random_location_update_)
                        new_position = update_location_random(proposed_position);
                else
                        new_position = update_location_cap(proposed_position);

                Arm new_arm = Arm(opti_func_, new_position);
                particles_[particle_index] = new_arm;
        }
}

void PSO::sample_and_update(int particle_index) {
        particles_[particle_index].pull();
        update_simulation_budget();

        if (new_local_best(particle_index))
                best_individual_arms_[particle_index] = particles_[particle_index];
        if (new_global_best(particle_index))
                best_particle_index_ = particle_index;
}

void PSO::optimize() {
        while (true) {
                for (int particle_index = 0; particle_index < num_particle_; particle_index++) {
                        sample_and_update(particle_index);

                        save_history();
                        if (budget_reached())
                                return;
                }

                update_positions();
        }
}

Eigen::VectorXi generate_unique_solution(std::vector<Eigen::VectorXi> &solutions, Eigen::VectorXi x_lb, Eigen::VectorXi x_ub, int dimension) {
        Eigen::VectorXi v(dimension);
        while (true) {
                for (int j = 0; j < dimension; j++) {
                        v(j) = random_uniform_int(x_lb(j), x_ub(j));
                }
                if (std::find(solutions.begin(), solutions.end(), v) == solutions.end())
                        break;
        }
        return v;
}

PSO::PSO(int num_particle, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max, std::function<double(Eigen::VectorXi, int)> opti_func, int max_simulation, bool use_random_location_update) : num_particle_{num_particle}, dimension_{dimension}, x_min_{std::move(x_min)}, x_max_{std::move(x_max)}, opti_func_{std::move(opti_func)}, max_simulations_{max_simulation}, use_random_location_update_{use_random_location_update}{
        std::vector<Eigen::VectorXi> init_solutions;
        for (int i = 0; i < num_particle_; i++) {
                velocity_.emplace_back(Eigen::VectorXd::Zero(dimension_));

                Eigen::VectorXi v(dimension_);
                v = generate_unique_solution(init_solutions, x_min_, x_max_, dimension_);
                init_solutions.push_back(v);

                particles_.emplace_back(opti_func_, v);
        }

        best_individual_arms_ = particles_;
}

void PSO::save_current_best_solution() {
        Arm& best_arm = best_individual_arms_[best_particle_index_];

        Eigen::VectorXi best_solution = best_arm.get_action_vector();
        double num_pulls_best = best_arm.num_pulls();
        double mean_value = best_arm.mean_reward();
        double true_value = best_arm.true_value();

        best_solutions_.emplace_back(simulations_used_, best_solution, num_pulls_best, mean_value, true_value);
}

int PSO::num_particle() const {
        return num_particle_;
}

int PSO::dimension() const {
        return dimension_;
}

std::vector<Arm> PSO::particles() const {
        return particles_;
}

std::vector<solution> &PSO::best_solutions() {
        return best_solutions_;
}

int PSO::simulations_used() const{
        return simulations_used_;
}

std::vector<Arm> &PSO::best_individual_arms() {
        return best_individual_arms_;
}

int &PSO::best_particle_index() {
        return best_particle_index_;
}

bool PSO::budget_reached() const{
        return simulations_used_ > max_simulations_;
}

void PSO::save_history() {
        if (simulations_used_ % 100 == 0)
                save_current_best_solution();
}

void PSO::update_simulation_budget(int number_of_new_simulations) {
        simulations_used_ += number_of_new_simulations;
}

bool PSO::new_local_best(int particle_index) {
        bool better_reward_observed = particles_[particle_index].mean_reward() < best_individual_arms_[particle_index].mean_reward();
        bool no_local_best_yet = best_individual_arms_[particle_index].reward() == 0;
        return better_reward_observed || no_local_best_yet;
}

bool PSO::new_global_best(int particle_index) {
        return best_individual_arms_[particle_index].mean_reward() < best_individual_arms_[best_particle_index_].mean_reward();
}

