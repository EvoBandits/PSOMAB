#include "Arm.h"
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <queue>
#include <thread>
#include <utility>
#include <vector>
#include "Eigen/Core"


void Arm::reset_arm() {
  r = 0.0;
  Q = 0.0;
  k = 0.0;
}

double Arm::pull_arm() {
  double G = (*this).function_value(get_action_vector());

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

Eigen::VectorXi Arm::get_action_vector() { return action_vector; }

void Arm::print_action_vector() {
  for (int i = 0; i < action_vector.size(); i++) {
    std::cout << action_vector[i] << " ";
  }
  std::cout << std::endl;
}

Arm::Arm(std::function<double(Eigen::VectorXi)> func, Eigen::VectorXi permutation, int calc_cost_info, double init_r,
         double init_k, double init_Q)
    : function_value{std::move(func)}, action_vector{std::move(permutation)},
      calc_cost_info{calc_cost_info}, r{init_r}, k{init_k}, Q{init_Q} {}

Arm::~Arm() = default;

