#ifndef PSOMAB_TESTS_PROBLEMS_F4_ACKLEY_H_
#define PSOMAB_TESTS_PROBLEMS_F4_ACKLEY_H_
#define _USE_MATH_DEFINES

#include "../../../memo/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"
#include <cmath>

double get_true_objective_value_ackley(const Eigen::VectorXi &action_vector);
double ackley(const Eigen::VectorXi &action_vector, bool noisy);
Eigen::VectorXd initialize_ackley_shift();
Eigen::MatrixXd initialize_ackley_rotation();

const Eigen::VectorXd ackley_shift = initialize_ackley_shift();
const Eigen::MatrixXd ackley_rotation = initialize_ackley_rotation();

#endif//PSOMAB_TESTS_PROBLEMS_F4_ACKLEY_H_
