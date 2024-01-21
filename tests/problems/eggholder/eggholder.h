#ifndef PSOMAB_TESTS_PROBLEMS_EGGHOLDER_H_
#define PSOMAB_TESTS_PROBLEMS_EGGHOLDER_H_

#include "../../../memo/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"

double get_true_objective_value_eggholder(const Eigen::VectorXi &action_vector);
double eggholder(const Eigen::VectorXi &action_vector, bool noisy);

#endif//PSOMAB_TESTS_PROBLEMS_EGGHOLDER_H_
