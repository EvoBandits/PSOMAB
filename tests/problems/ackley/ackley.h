#ifndef PSOMAB_TESTS_PROBLEMS_ACKLEY_H_
#define PSOMAB_TESTS_PROBLEMS_ACKLEY_H_

#include "../../../memo/util/RandomNumber.h"
#include "Eigen/Core"

extern Eigen::VectorXi ackley_lb;
extern Eigen::VectorXi ackley_ub;
extern int ackley_dim;
extern double ackley_step_size;

extern double ackley_noise_level;

double get_true_objective_value_ackley(const Eigen::VectorXi &action_vector);
double ackley(const Eigen::VectorXi &action_vector, bool noisy);

#endif//PSOMAB_TESTS_PROBLEMS_ACKLEY_H_
