#ifndef PSOMAB_PSOMAB_RANDOMNUMBER_H_
#define PSOMAB_PSOMAB_RANDOMNUMBER_H_

#include <random>

extern thread_local std::default_random_engine generator;

void seed_generator(unsigned int seed);

int random_uniform_int(int a, int b);
double random_uniform_double(double a, double b);
int random_poisson(double a);
double random_normal(double mean, double stddev);

#endif//PSOMAB_PSOMAB_RANDOMNUMBER_H_
