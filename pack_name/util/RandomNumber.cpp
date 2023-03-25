#include "RandomNumber.h"

std::random_device rd;// obtain a random number from hardware
std::default_random_engine generator(111);


int random_uniform_int(int a, int b) {
        std::uniform_int_distribution<int> random_value(a,b);
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