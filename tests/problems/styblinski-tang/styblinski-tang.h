#ifndef PSOMAB_TESTS_PROBLEMS_STYBLINSKI_TANG_STYBLINSKI_TANG_H_
#define PSOMAB_TESTS_PROBLEMS_STYBLINSKI_TANG_STYBLINSKI_TANG_H_

#include "../../../memo/util/RandomNumber.h"
#include "../../problem_parameter.h"
#include "Eigen/Core"
#include <random>
#include <vector>

double get_true_objective_value(const Eigen::VectorXi &action_vector);
double styblinski_tang(const Eigen::VectorXi &action_vector, bool noisy);

#endif//PSOMAB_TESTS_PROBLEMS_STYBLINSKI_TANG_STYBLINSKI_TANG_H_
