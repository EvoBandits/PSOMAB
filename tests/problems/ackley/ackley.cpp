#include "ackley.h"
#include <fstream>
#include <nlohmann/json.hpp>

nlohmann::json configA = nlohmann::json::parse(std::ifstream("config.json"));

int ackley_dim = configA["problems"]["ackley"]["DIMENSIONS"];
double ackley_step_size = configA["problems"]["ackley"]["STEP_SIZE"];
Eigen::VectorXi ackley_lb = Eigen::VectorXi::Constant(ackley_dim, configA["problems"]["ackley"]["LOWER_BOUND"]);
Eigen::VectorXi ackley_ub = Eigen::VectorXi::Constant(ackley_dim, configA["problems"]["ackley"]["UPPER_BOUND"]);

double ackley_noise_level = configA["problems"]["ackley"]["NOISE_LEVEL"];

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
