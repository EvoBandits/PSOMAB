#ifndef PSOMAB_PACK_NAME_PSOMAB_SOLUTION_H_
#define PSOMAB_PACK_NAME_PSOMAB_SOLUTION_H_

#include "Eigen/Core"
#include <algorithm>
class solution {
       public:
        int obs_number;   // the "obs-number"-th observation over the course of time
        Eigen::VectorXi x;// x
        int N;
        double mean_func_val = 0.0;// mean function value of gbest
        double true_func_val = 0.0;// true function value of gbest
        double average_true_func_val = 0.0;

        solution(int obs_number, Eigen::VectorXi x, int N, double mean_func_val,
                 double true_func_val)
            : obs_number(obs_number), x(std::move(x)), N(N), mean_func_val(mean_func_val),
              true_func_val(true_func_val) {}
        void print() const;
};

#endif//PSOMAB_PACK_NAME_PSOMAB_SOLUTION_H_
