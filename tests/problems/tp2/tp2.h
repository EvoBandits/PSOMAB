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

double poisson_random_number_tp2(double a);
int uniform_random_number_tp2(int a, int b);
void calc_inventory_tp2(int start_inventory[][8][2]);
int calc_TC_tp2(int start_inventory[][8][2]);

/*
int main() {
        std::cout << std::fixed << std::setprecision(0);

        std::vector<int> min={191,  40};  // min x_1   // min x_1
        std::vector<int> max={360,  300};  // max x_1   // max x_2

        std::vector<std::vector<int>> action_vectors;
        std::vector<double> result_vector_mean;
        std::vector<double> result_vector_stdev;

        for(int i=min[0]; i<=max[0]; i++){  //klein s
                for(int j=min[1]; j<=max[1]; j++){ // GROß S
                        generator.seed(1); //standardmäßig bei jedem Durchlauf auf 1
                        double final_r=0.0;
                        std::vector<double> results;
                        int sample_size=10'000;
                        for(int o=0; o<sample_size; o++) {
                                std::vector<int> s{i,j};

                                int start_inventory [2][8][2]= { {{s[0],s[1]},   {0,0},    {0,0},    {0,0},    {0,0},    {0,0},  {0,0},   {0,0},},
                                                                {{0,0},   {0,0},    {0,0},    {0,0},    {0,0},    {0,0},  {0,0},   {0,0}}   };

                                int period_number=1'200; // Supply Chain Horizon
                                double reward{0.0};
                                for(int t=0; t<period_number; t++){
                                        calc_inventory_tp2(start_inventory);
                                        reward=reward+calc_TC_tp2(start_inventory);
                                }
                                results.push_back(reward);
                        }
                        double sum = std::accumulate(results.begin(), results.end(), 0.0);
                        double mean = sum / results.size();

                }
        }


        return 0;
}
*/

#endif//PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_
