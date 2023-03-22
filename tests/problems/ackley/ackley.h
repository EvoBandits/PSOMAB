#ifndef PSOMAB_TESTS_PROBLEMS_ACKLEY_H_
#define PSOMAB_TESTS_PROBLEMS_ACKLEY_H_

#include "Eigen/Core"
#include "../../../pack_name/util/RandomNumber.h"

extern Eigen::Vector2i ackley_lb;
extern Eigen::Vector2i ackley_ub;
extern int ackley_dim;

double get_true_objective_value_ackley(const Eigen::VectorXi& action_vector);
double ackley(Eigen::VectorXi action_vector, int noise_level);

#endif//PSOMAB_TESTS_PROBLEMS_ACKLEY_H_
