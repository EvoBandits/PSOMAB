#include "f2_six_hump_camel_back.h"

double get_true_objective_value_six_hump_camel_back(const Eigen::VectorXi &action_vector) {
        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * step_size;

        const double x = action_vector_scaled[0];
        const double y = action_vector_scaled[1];

        double result = 4 * std::pow(x, 2)
            - 2.1 * std::pow(x, 4)
            + (1.0 / 3.0) * std::pow(x, 6)
            + x * y
            - 4 * std::pow(y, 2)
            + 4 * std::pow(y, 4)
            + 1.0315704;

        // scale value range to approx. [0;100]
        result *= 15;

        return result;
}

double six_hump_camel_back(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_six_hump_camel_back(action_vector) + random_normal(0, noisy * noise_level);
}
