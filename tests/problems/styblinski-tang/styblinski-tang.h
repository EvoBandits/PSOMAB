#ifndef PSOMAB_TESTS_PROBLEMS_STYBLINSKI_TANG_STYBLINSKI_TANG_H_
#define PSOMAB_TESTS_PROBLEMS_STYBLINSKI_TANG_STYBLINSKI_TANG_H_

#include "../../../memo/util/RandomNumber.h"
#include "Eigen/Core"
#include <random>
#include <vector>

extern Eigen::VectorXi styblinski_tang_lb;
extern Eigen::VectorXi styblinski_tang_ub;
extern int styblinski_tang_dim;

double get_true_objective_value(const Eigen::VectorXi& action_vector);
double styblinski_tang(const Eigen::VectorXi& action_vector, int noise_level);

#endif//PSOMAB_TESTS_PROBLEMS_STYBLINSKI_TANG_STYBLINSKI_TANG_H_
