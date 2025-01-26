#include "styblinski-tang.h"

double get_true_objective_value_styblinski_tang(const Eigen::VectorXi &action_vector) {
        double sum = 0.0;
        constexpr double a = 16.0;
        constexpr double b = 5.0;

        const Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * step_size;

        for (const auto x : action_vector_scaled)
                sum += pow(x, 4) - a * pow(x, 2) + b * x;

        return sum / 2.0;
}
double styblinski_tang(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_styblinski_tang(action_vector) + random_normal(0, noisy * noise_level);
}
