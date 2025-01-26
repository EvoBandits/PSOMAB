#ifndef PSOMAB_TESTS_PROBLEMS_SIX_HUMP_CAMEL_BACK_H_
#define PSOMAB_TESTS_PROBLEMS_SIX_HUMP_CAMEL_BACK_H_

#include "../../../memo/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"

double get_true_objective_value_six_hump_camel_back(const Eigen::VectorXi &action_vector);
double six_hump_camel_back(const Eigen::VectorXi &action_vector, bool noisy);

#endif//PSOMAB_TESTS_PROBLEMS_SIX_HUMP_CAMEL_BACK_H_
