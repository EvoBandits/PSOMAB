#include "ackley.h"

int ackley_dim = 10;
double ackley_step_size = 0.01;
Eigen::VectorXi ackley_lb = Eigen::VectorXi::Constant(ackley_dim, -2000);
Eigen::VectorXi ackley_ub = Eigen::VectorXi::Constant(ackley_dim, 2000);

double ackley_noise_level = 2;

double get_true_objective_value_ackley(const Eigen::VectorXi &action_vector) {

        double a = 20;
        double b = 0.2;

        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * ackley_step_size;

        double sum_sq = action_vector_scaled.array().square().sum() / 100.0;
        double sum_cos = (action_vector_scaled.array().cos()).sum();

        return -a * exp(-b * sqrt(sum_sq / ackley_dim)) - exp(sum_cos / ackley_dim) + a + exp(1);
}

double ackley(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_ackley(action_vector) + random_normal(0, ackley_noise_level * noisy);
}
