#include "Arm.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <queue>
#include <random>
#include <thread>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

unsigned seed3 = 1; //
std::default_random_engine e3(seed3);

double normal_random_number(double m, double s) {
  std::normal_distribution<double> random_normal(m, s);
  return random_normal(e3);
};



int random_demand(int a, int b) {
  std::uniform_int_distribution<int> random_integer(a, b);
  return random_integer(e3);
};

double double_random_number(double a, double b) {
  std::uniform_real_distribution<double> random_integer(a, b);
  return random_integer(e3);
};

struct PQ_element {

  int customer_index;
  bool demand;
  int product_index;
  double timestep;

  // this will used to initialize the variables
  // of the structure

  PQ_element(int customer_index, bool demand, int product_index,
             double timestep)
      : customer_index(customer_index), demand(demand),
        product_index(product_index), timestep(timestep) {}
};

struct Compare_timestep {
  bool operator()(PQ_element &a1, PQ_element &a2) {
    return a1.timestep > a2.timestep; // > (smallest elements at the top)
  }
};

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
  for (int i = 0; i < action_vector.size(); i++) {
    std::cout << action_vector[i] << " ";
  }
  std::cout << std::endl;
}

Arm::Arm(std::function<double(std::vector<int>)> func, std::vector<int> permutation, int calc_cost_info, double init_r,
         double init_k, double init_Q)
    : arm_function{std::move(func)}, action_vector{permutation},
      calc_cost_info{calc_cost_info}, r{init_r}, k{init_k}, Q{init_Q} {}

Arm::~Arm() {}
