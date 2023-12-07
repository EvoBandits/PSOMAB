#include "styblinski-tang.h"

int styblinski_tang_dim = 10;
Eigen::VectorXi styblinski_tang_lb = Eigen::VectorXi::Constant(styblinski_tang_dim, -5000);
Eigen::VectorXi styblinski_tang_ub = Eigen::VectorXi::Constant(styblinski_tang_dim, 5000);
double styblinski_tang_step_size = 0.001;

double styblinski_tang_noise_level = 1.5;

double get_true_objective_value_styblinski_tang(const Eigen::VectorXi& action_vector) {
        double sum = 0.0;
        const double a = 16.0;
        const double b = 5.0;

        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * styblinski_tang_step_size;

        for (const auto x : action_vector_scaled)
                sum += pow(x, 4) - a * pow(x, 2) + b * x;

        return sum / 2.0;
}
double styblinski_tang(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_styblinski_tang(action_vector) + random_normal(0, noisy * styblinski_tang_noise_level);
}