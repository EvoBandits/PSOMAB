#ifndef PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_
#define PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_

#include "../../memo/util/RandomNumber.h"
#include "Eigen/Core"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

extern Eigen::Vector4i tp2_lb;
extern Eigen::Vector4i tp2_ub;
constexpr int tp2_dim = 4;

void calc_inventory_tp2(int start_inventory[][8][4]);
int calc_TC_tp2(int start_inventory[][8][4]);
double get_true_objective_value_tp2(const Eigen::VectorXi &action_vector);
double tp2(const Eigen::VectorXi &action_vector, int noise_level);

#endif//PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_
