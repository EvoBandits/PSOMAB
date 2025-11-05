#include "SmartRounding.h"

Eigen::VectorXi smart_rounding(Eigen::VectorXd &v, int desired_sum) {
        Eigen::VectorXd margin(v.size());
        Eigen::VectorXi rounded(v.size());

        for (int index = 0; index < v.size(); index++) {
                rounded(index) = std::floor(v(index));
                margin(index) = v(index) - rounded(index);
        }

        std::vector<int> indices = sort_indices(margin);
        std::reverse(indices.begin(), indices.end());
        indices.resize(desired_sum - rounded.sum());

        for (int index : indices)
                rounded(index) += 1;

        return rounded;
}
