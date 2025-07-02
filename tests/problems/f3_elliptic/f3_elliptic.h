#ifndef PSOMAB_TESTS_PROBLEMS_F3_ELLIPTIC_H_
#define PSOMAB_TESTS_PROBLEMS_F3_ELLIPTIC_H_

#include "../../../src/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"

double get_true_objective_value_elliptic(const Eigen::VectorXi &action_vector);
double elliptic(const Eigen::VectorXi &action_vector, bool noisy);
Eigen::VectorXd initialize_elliptic_shift();
Eigen::MatrixXd initialize_elliptic_rotation();

const Eigen::VectorXd elliptic_shift = initialize_elliptic_shift();
const Eigen::MatrixXd elliptic_rotation = initialize_elliptic_rotation();

#endif//PSOMAB_TESTS_PROBLEMS_F3_ELLIPTIC_H_
