#ifndef MEMO_MICHALEWICZ_SPHERE_H
#define MEMO_MICHALEWICZ_SPHERE_H

#define _USE_MATH_DEFINES
#include <cmath>
#include "../../../memo/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"

double get_true_objective_value_michalewicz_sphere(const Eigen::VectorXi &action_vector);
double michalewicz_sphere(const Eigen::VectorXi &action_vector, bool noisy);

#endif//MEMO_MICHALEWICZ_SPHERE_H
