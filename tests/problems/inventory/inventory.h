#ifndef PSOMAB_TESTS_PROBLEMS_INVENTORY_H_
#define PSOMAB_TESTS_PROBLEMS_INVENTORY_H_

#include "../../../src/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"
#include <random>
#include <vector>

// 17 36

double get_true_objective_value(const Eigen::VectorXi &action_vector);
double inventory(const Eigen::VectorXi &action_vector, bool noisy);

#endif// PSOMAB_TESTS_PROBLEMS_INVENTORY_H_
