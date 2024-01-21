#include "eggholder.h"

double get_true_objective_value_eggholder(const Eigen::VectorXi &action_vector) {
        Eigen::VectorXd action_vector_scaled = action_vector.cast<double>() * step_size;

        double result = -(action_vector_scaled[1] + 47) * sin(sqrt(abs(action_vector_scaled[0] / 2 + (action_vector_scaled[1] + 47)))) - action_vector_scaled[0] * sin(sqrt(abs(action_vector_scaled[0] - (action_vector_scaled[1] + 47))));
        return result;
}

double eggholder(const Eigen::VectorXi &action_vector, bool noisy) {
        return get_true_objective_value_eggholder(action_vector) + random_normal(0, noise_level * noisy);
}
