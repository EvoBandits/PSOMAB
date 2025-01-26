#include "f1_rosenbrock.h"

double get_true_objective_value_rosenbrock(const Eigen::VectorXi &action_vector) {
        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * step_size;

        constexpr int a = 1;
        constexpr int b = 100;

        const double x = action_vector_scaled[0];
        const double y = action_vector_scaled[1];

        const double result = (pow(a - x, 2) + b * pow(y - pow(x, 2), 2)) / 25.0;

        return result;
}

double rosenbrock(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_rosenbrock(action_vector) + random_normal(0, noisy * noise_level);
}
