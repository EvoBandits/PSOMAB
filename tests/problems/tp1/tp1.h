#ifndef PSOMAB_TESTS_PROBLEMS_TP1_TP1_H_
#define PSOMAB_TESTS_PROBLEMS_TP1_TP1_H_

#include <iostream>
#include <random>
#include <vector>
#include <iomanip>
#include <numeric>
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


int random_number(int a, int b){
        std::uniform_int_distribution<int> random_integer(a,b);
        return random_integer(generator);
};

void calc_inventory(int start_inventory[][8][2]);
int calc_TC(int start_inventory[][8][2]);
double inventory();
double Variance(std::vector<double> samples);

int main(){

        std::vector<double> results;
        int sample_size=10'000;
        for(int i=1; i<=sample_size;i++){

                // best base-stock level vector
                std::vector<int> s{53,192}; //SSS1

                int start_inventory [2][8][2]= { {{s[0],s[1]}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0},},
                                                 {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}}};

                // start_inventory [0][0][1],...,[0][0][4] denote the inventory levels of agent 1,...,4
                /*---------------------------------------*/
                // start_inventory [0][1][1],...,[0][1][4]
                // start_inventory ....................... denote the shipments that are in transit
                // start_inventory [0][7][1],...,[0][7][4]
                /*---------------------------------------*/
                // start_inventory [1][0][1],...,[1][0][4]
                // start_inventory ....................... denote the orders that are in transit
                // start_inventory [1][7][1],...,[1][7][4]

                int period_number=1'200; // Supply Chain Horizon
                double reward{0.0};
                for(int i=0; i<period_number; i++){
                        calc_inventory(start_inventory);
                        reward=reward+calc_TC(start_inventory);
                }
                results.push_back(reward);
        }

        double sum = std::accumulate(results.begin(), results.end(), 0.0);
        double mean = sum / results.size();

        double var = Variance(results);
        double stdev = sqrt(var);

        std::cout << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "mean TC: " << mean << "    sample size: "<<sample_size<<""<< std::endl;
        std::cout << "std: " << stdev << "  ||  variance: " << var << "    sample size: "<<sample_size<<""<< std::endl;

        std::getchar();
        return 0;
}


#endif//PSOMAB_TESTS_PROBLEMS_TP1_TP1_H_
