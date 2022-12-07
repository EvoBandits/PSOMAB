#include "Arm.h"
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

void Arm::reset_arm() {
        r = 0.0;
        Q = 0.0;
        k = 0.0;
}

double Arm::function_value() {
        return arm_function(action_vector);
}

double Arm::pull_arm() {
        double G = (*this).function_value();

        r = r + G;
        k = k + 1;

        return G;
}

double Arm::get_Q() { return Q; }

double Arm::get_k() { return k; }

double Arm::get_r() { return r; }

double Arm::get_mu() { return mu; }

void Arm::update_k(int k_var) { k = k + k_var; }

void Arm::update_r(double r_var) { r = r + r_var; }

void Arm::set_action_vector_element(int index, int value) {
        action_vector[index] = value;
}

std::vector<int> Arm::get_action_vector() { return action_vector; }

void Arm::print_action_vector() {
        for (int i : action_vector) {
                std::cout << i << " ";
        }
        std::cout << std::endl;
}

Arm::Arm(std::function<double(std::vector<int>)> func, std::vector<int> permutation, double init_r,
         double init_k, double init_Q)
    : arm_function{std::move(func)}, action_vector{std::move(permutation)}, r{init_r}, k{init_k}, Q{init_Q} {}

Arm::~Arm() {}
