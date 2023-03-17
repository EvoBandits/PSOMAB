#include "SolutionCodeCalculation.h"

// calculates the solution code, i.e. unique integer
boost::multiprecision::int128_t calc_solution_code(Eigen::VectorXi action_vector, int dimension, Eigen::VectorXi x_min, Eigen::VectorXi x_max) {
        boost::multiprecision::int128_t search_index = 0;
        for (int i = 0; i < dimension; i++) {
                int exp = ceil(log10((x_max[i] - x_min[i]) + 1));
                search_index = search_index + (boost::multiprecision::int128_t) (pow((pow(10, exp)), (dimension - 1) - i) * (action_vector[i] - x_min[i]));
        }
        return search_index;
}
