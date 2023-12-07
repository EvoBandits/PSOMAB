#ifndef PSOMAB_TESTS_PROBLEMS_INVENTORY_H_
#define PSOMAB_TESTS_PROBLEMS_INVENTORY_H_

#include "../../../memo/util/RandomNumber.h"
#include "Eigen/Core"
#include <random>
#include <vector>

// 17 36
extern Eigen::Vector2i inventory_lb;
extern Eigen::Vector2i inventory_ub;
extern int inventory_dim;

extern double inventory_noise_level;

double get_true_objective_value(const Eigen::VectorXi& action_vector);
double inventory(Eigen::VectorXi action_vector, bool noisy);

#endif// PSOMAB_TESTS_PROBLEMS_INVENTORY_H_
