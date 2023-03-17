#ifndef PSOMAB_PACK_NAME_UTIL_SOLUTIONCODECALCULATION_H_
#define PSOMAB_PACK_NAME_UTIL_SOLUTIONCODECALCULATION_H_

#include <boost/multiprecision/cpp_int.hpp>
#include <Eigen/Core>

boost::multiprecision::int128_t calc_solution_code(Eigen::VectorXi action_vector, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max);

#endif//PSOMAB_PACK_NAME_UTIL_SOLUTIONCODECALCULATION_H_
