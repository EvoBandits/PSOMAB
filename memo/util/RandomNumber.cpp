#include "RandomNumber.h"

thread_local std::default_random_engine generator;

void seed_generator(unsigned int seed) {
        generator.seed(seed);
}

int random_uniform_int(int a, int b) {
        std::uniform_int_distribution<int> random_value(a, b);
        return random_value(generator);
}

double random_uniform_double(double a, double b) {
        std::uniform_real_distribution<double> random_value(a, b);
        return random_value(generator);
}

int random_poisson(double mean) {
        std::poisson_distribution<int> random_value(mean);
        return random_value(generator);
}

double random_normal(double mean, double stddev) {
        std::normal_distribution<double> random_value(mean, stddev);
        return random_value(generator);
}
