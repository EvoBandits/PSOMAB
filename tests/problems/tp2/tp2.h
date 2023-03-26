#ifndef PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_
#define PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_

#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>
#include "Eigen/Core"
#include "../../pack_name/util/RandomNumber.h"

extern Eigen::Vector4i tp2_lb;
extern Eigen::Vector4i tp2_ub;
extern int tp2_dim;

void calc_inventory_tp2(int start_inventory[][8][4]);
int calc_TC_tp2(int start_inventory[][8][4]);
double tp2(Eigen::VectorXi action_vector, int noise_level);


#endif//PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_
