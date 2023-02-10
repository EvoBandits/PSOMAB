#include "Arm.h"
#include <algorithm>
#include <utility>

double Arm::function_value() const {
        return arm_func_(action_vector_);
}

double Arm::pull() {
        double G = (*this).function_value();

        reward_ = reward_ + G;
        num_pulls_ = num_pulls_ + 1;

        return G;
}

double Arm::get_Q() const { return Q_; }

double Arm::num_pulls() const { return num_pulls_; }

double Arm::reward() const { return reward_; }

void Arm::update_num_pulls(int k_var) { num_pulls_ = num_pulls_ + k_var; }

void Arm::update_reward(double r_var) { reward_ = reward_ + r_var; }

void Arm::set_action_vector_element(int index, int value) {
        action_vector_[index] = value;
}

Eigen::VectorXi Arm::get_action_vector() const { return action_vector_; }

Arm::Arm(std::function<double(Eigen::VectorXi)> func, Eigen::VectorXi permutation, double init_r,
         double init_k, double init_Q)
    : arm_func_{std::move(func)}, action_vector_{std::move(permutation)}, reward_{init_r}, num_pulls_{init_k}, Q_{init_Q} {}
