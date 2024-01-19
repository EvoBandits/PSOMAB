#include "ackley.h"

double get_true_objective_value_ackley(const Eigen::VectorXi &action_vector) {

        double a = 20;
        double b = 0.2;

        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * step_size;

        double sum_sq = action_vector_scaled.array().square().sum() / 100.0;
        double sum_cos = (action_vector_scaled.array().cos()).sum();

        return -a * exp(-b * sqrt(sum_sq / dimension)) - exp(sum_cos / dimension) + a + exp(1);
}

double ackley(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_ackley(action_vector) + random_normal(0, noise_level * noisy);
}
