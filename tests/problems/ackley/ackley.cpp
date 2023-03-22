#include "ackley.h"

Eigen::Vector2i ackley_lb(-500, -500);
Eigen::Vector2i ackley_ub(500, 500);
int ackley_dim = 2;

double get_true_objective_value_ackley(const Eigen::VectorXi &action_vector) {
        double sum_sq = 0;
        double sum_cos = 0;

        double a = 20;
        double b = 0.2;
        double c = 2 * M_PI;

        for (int i = 0; i < ackley_dim; i++) {
                sum_sq += pow(action_vector[i], 2)/100;
                sum_cos += cos(c * action_vector[i]);
        }

        return -a*exp(-b*sqrt(sum_sq/ackley_dim)) - exp(sum_cos/ ackley_dim) + a + exp(1);
}


double ackley(Eigen::VectorXi action_vector, int noise_level) {
        return get_true_objective_value_ackley(action_vector) + noise_level * random_normal(0, 1);
}

