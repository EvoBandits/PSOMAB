#include "styblinski-tang.h"

int styblinski_tang_dim = 10;
Eigen::VectorXi styblinski_tang_lb = Eigen::VectorXi::Constant(styblinski_tang_dim, -5);
Eigen::VectorXi styblinski_tang_ub = Eigen::VectorXi::Constant(styblinski_tang_dim, 5);

double get_true_objective_value_styblinski_tang(const Eigen::VectorXi& action_vector) {
        double sum = 0.0;
        const double a = 16.0;
        const double b = 5.0;

        for (const auto x : action_vector)
                sum += pow(x, 4) - a * pow(x, 2) + b * x;

        return sum / 2.0;
}
double styblinski_tang(const Eigen::VectorXi& action_vector, int noise_level) {
        return get_true_objective_value_styblinski_tang(action_vector) + noise_level * random_normal(0, 1);
}