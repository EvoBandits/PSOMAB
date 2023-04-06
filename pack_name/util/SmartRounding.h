#ifndef PSOMAB_PACK_NAME_UTIL_SMARTROUNDING_H_
#define PSOMAB_PACK_NAME_UTIL_SMARTROUNDING_H_

#include "Eigen/Core"
#include "SortIndices.h"

Eigen::VectorXi smart_rounding(Eigen::VectorXd &v, int desired_sum);

#endif//PSOMAB_PACK_NAME_UTIL_SMARTROUNDING_H_
