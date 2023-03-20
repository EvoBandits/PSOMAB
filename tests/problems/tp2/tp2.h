#ifndef PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_
#define PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_

#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>
#include "../../pack_name/util/RandomNumber.h"

int d_min=20;
int d_max=60;

// backorder costs
std::vector<int> b_c{24,12,6,3};
// holding costs
std::vector<int> h_c{8,4,2,1};

//transportation lead times
std::vector<int> tlt_min{1,2,4,3};
std::vector<int> tlt_max{1,4,6,5};

//information lead times
std::vector<int> ilt_min{0,0,0,0};
std::vector<int> ilt_max{0,1,1,2};


void calc_inventory(int start_inventory[][8][2]);
int calc_TC(int start_inventory[][8][2]);
double Variance(std::vector<double> samples);


double poisson_random_number(double a){
        std::poisson_distribution<int> random_integer(a);
        return random_integer(generator);
};

int random_number(int a, int b){
        std::uniform_int_distribution<int> random_integer(a,b);
        return random_integer(generator);
};


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
                                        calc_inventory(start_inventory);
                                        reward=reward+calc_TC(start_inventory);
                                }
                                results.push_back(reward);
                        }
                        double sum = std::accumulate(results.begin(), results.end(), 0.0);
                        double mean = sum / results.size();

                }
        }


        return 0;
}

#endif//PSOMAB_TESTS_PROBLEMS_TP2_TP2_H_
