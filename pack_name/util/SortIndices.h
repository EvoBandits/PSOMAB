#ifndef PSOMAB_PACK_NAME_UTIL_SORTINDICES_H_
#define PSOMAB_PACK_NAME_UTIL_SORTINDICES_H_

#include <Eigen/Core>

bool sort_indices_comp(const std::pair<int, double> & left, const std::pair<int, double> & right);
std::vector<int> sort_indices(const Eigen::VectorXd &x);

#endif//PSOMAB_PACK_NAME_UTIL_SORTINDICES_H_
