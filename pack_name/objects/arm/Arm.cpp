#include "Arm.h"
#include <algorithm>
#include <utility>

double Arm::function_value() const {
        return arm_func_(action_vector_, 1);
}

double Arm::true_value() const {
        return arm_func_(action_vector_, 0);
}

double Arm::pull() {
        double G = (*this).function_value();

        reward_ = reward_ + G;
        reward_squared_ = reward_squared_ + G * G;
        num_pulls_ = num_pulls_ + 1;

        return G;
}

int Arm::num_pulls() const { return num_pulls_; }

double Arm::reward() const { return reward_; }

void Arm::update_num_pulls(int new_num_pulls) {
        num_pulls_ = num_pulls_ + new_num_pulls;
}

void Arm::update_reward(double new_reward) {
        reward_ = reward_ + new_reward;
        reward_squared_ = reward_squared_ + new_reward * new_reward;
}

Eigen::VectorXi Arm::get_action_vector() const { return action_vector_; }

double Arm::mean_reward() const {
        return reward_ / num_pulls_;
}

Arm::Arm(std::function<double(Eigen::VectorXi, int)> func, Eigen::VectorXi action_vector, double initial_reward, int initial_num_pulls)
    : arm_func_{std::move(func)}, action_vector_{std::move(action_vector)}, reward_{initial_reward}, num_pulls_{initial_num_pulls}, reward_squared_{initial_reward*initial_reward} {}

double Arm::variance() const {
        if (num_pulls_ <= 1)
                return 0;
        else
                return (reward_squared_ - reward_ * reward_ / num_pulls_) / (num_pulls_-1);
}
