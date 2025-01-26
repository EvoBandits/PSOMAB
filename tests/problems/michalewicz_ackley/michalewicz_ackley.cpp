#include "michalewicz_ackley.h"

double get_true_objective_value_michalewicz_ackley(const Eigen::VectorXi &action_vector) {
        // Ackley function parameters
        constexpr double a = 20;
        constexpr double b = 0.2;
        constexpr double c = 2 * M_PI;

        // Michalewicz function parameters
        constexpr double m = 10;

        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * step_size;

        // Ackley function calculation
        const double sum_sq = action_vector_scaled.array().square().sum();
        const double sum_cos = (action_vector_scaled.array() * c).cos().sum();
        const double ackley_part = -a * exp(-b * sqrt(sum_sq / dimension)) - exp(sum_cos / dimension) + a + exp(1);

        // Michalewicz function calculation
        double michalewicz_part = 0;
        for (int i = 0; i < dimension; ++i) {
                michalewicz_part -= sin(action_vector_scaled[i]) * pow(sin((i + 1) * action_vector_scaled[i] * action_vector_scaled[i] / M_PI), 2 * m);
        }

        return 0.2 * ackley_part + michalewicz_part;
}

double michalewicz_ackley(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_michalewicz_ackley(action_vector) + random_normal(0, noise_level * noisy);
}
