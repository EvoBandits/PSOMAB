#ifndef PSOMAB_TESTS_PROBLEMS_ROSENBROCK_H_
#define PSOMAB_TESTS_PROBLEMS_ROSENBROCK_H_

#include "../../../memo/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"

double get_true_objective_value_rosenbrock(const Eigen::VectorXi &action_vector);
double rosenbrock(const Eigen::VectorXi &action_vector, bool noisy);

#endif//PSOMAB_TESTS_PROBLEMS_ROSENBROCK_H_
