#include "michalewicz_sphere.h"

double get_true_objective_value_michalewicz_sphere(const Eigen::VectorXi &action_vector) {
        // Parameters for Michalewicz Function
        constexpr double m = 10;

        // Sphere Function
        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * step_size;
        const double sphere_sum = action_vector_scaled.array().square().sum();

        // Michalewicz Function
        double michalewicz_sum = 0;
        for (int i = 0; i < action_vector.size(); ++i) {
                michalewicz_sum += sin(action_vector_scaled[i]) * pow(sin((i + 1) * action_vector_scaled[i] * action_vector_scaled[i] / M_PI), 2 * m);
        }

        return -1 * michalewicz_sum + 0.1 * sphere_sum;
}

double michalewicz_sphere(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_michalewicz_sphere(action_vector) + random_normal(0, noise_level * noisy);
}
