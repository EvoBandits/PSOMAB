#ifndef PSOMAB_TESTS_PROBLEMS_TP1_TP1_H_
#define PSOMAB_TESTS_PROBLEMS_TP1_TP1_H_

#include "../../pack_name/util/RandomNumber.h"
#include "Eigen/Core"
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

extern Eigen::Vector2i tp1_lb;
extern Eigen::Vector2i tp1_ub;
constexpr int tp1_dim = 2;

void calc_inventory_tp1(int start_inventory[][8][2]);
int calc_TC_tp1(int start_inventory[][8][2]);
double get_true_objective_value_tp1(const Eigen::VectorXi &action_vector);
double tp1(const Eigen::VectorXi &action_vector, int noise_level);

#endif//PSOMAB_TESTS_PROBLEMS_TP1_TP1_H_
