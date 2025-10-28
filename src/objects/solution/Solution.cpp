#include "Solution.h"
#include <iostream>

void solution::print() const {
        std::cout << "budget:" << obs_number << "\t| true:" << true_func_val << "\tmean:" << mean_func_val << "\tN: " << N << "\t[" << x.transpose() << "]" << std::endl;
}
