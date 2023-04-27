#include "ackley.h"

int ackley_dim = 1000;
Eigen::VectorXi ackley_lb = Eigen::VectorXi::Constant(ackley_dim, -500);
Eigen::VectorXi ackley_ub = Eigen::VectorXi::Constant(ackley_dim, 500);

double get_true_objective_value_ackley(const Eigen::VectorXi &action_vector) {

        double a = 20;
        double b = 0.2;

        double sum_sq = action_vector.array().square().sum() / 100.0;
        double sum_cos = (action_vector.array().cos()).sum();

        return -a * exp(-b * sqrt(sum_sq / ackley_dim)) - exp(sum_cos / ackley_dim) + a + exp(1);
}

double ackley(const Eigen::VectorXi &action_vector, int noise_level) {
        if (noise_level)
                return get_true_objective_value_ackley(action_vector) * random_normal(1, 0.18);
        else
                return get_true_objective_value_ackley(action_vector);
}
