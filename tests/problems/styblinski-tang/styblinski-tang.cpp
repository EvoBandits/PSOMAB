#include "styblinski-tang.h"
#include <fstream>
#include <nlohmann/json.hpp>

nlohmann::json configS = nlohmann::json::parse(std::ifstream("config.json"));

int styblinski_tang_dim = configS["problems"]["styblinski-tang"]["DIMENSIONS"];
Eigen::VectorXi styblinski_tang_lb = Eigen::VectorXi::Constant(styblinski_tang_dim, configS["problems"]["styblinski-tang"]["LOWER_BOUND"]);
Eigen::VectorXi styblinski_tang_ub = Eigen::VectorXi::Constant(styblinski_tang_dim, configS["problems"]["styblinski-tang"]["UPPER_BOUND"]);
double styblinski_tang_step_size = configS["problems"]["styblinski-tang"]["STEP_SIZE"];

double styblinski_tang_noise_level = configS["problems"]["styblinski-tang"]["NOISE_LEVEL"];

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