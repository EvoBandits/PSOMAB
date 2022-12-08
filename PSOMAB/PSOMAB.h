#ifndef _PSOMAB_H_
#define _PSOMAB_H_
#include "Arm.h"
#include "LUT.h"//
#include <boost/multiprecision/cpp_int.hpp>
#include <cmath>
#include <queue>
#include <random>
#include <set>
#include <vector>

using namespace boost::multiprecision;

///////

struct MS_element {
        int arm_index;
        double Q;

        bool operator<(const MS_element &rhs) const { return Q < rhs.Q; }
        bool operator>(const MS_element &rhs) const { return Q > rhs.Q; }

        MS_element(int arm_index, double Q) : arm_index(arm_index), Q(Q) {}
};

struct solution {
        int obs_number;    // the "obs-number"-th observation over the course of time
        std::vector<int> x;// x
        int N;
        double mean_func_val = 0.0;// meanfunction value
        double true_func_val = 0.0;// truefunction value

        solution(int obs_number, std::vector<int> x, int N, double mean_func_val,
                 double true_func_val)
            : obs_number(obs_number), x(x), N(N), mean_func_val(mean_func_val),
              true_func_val(true_func_val) {}
};

class PSOMAB {
       private:
        unsigned long max_iter_or_sim_number;// ToDo ???
        int m;                               // ToDo ???

        std::vector<int> vec_x_min;// D-dimensional Vector of the smallest possible
                                   // values a solution can have --> lower bounds
        std::vector<int> vec_x_max;// D-Dimensional Vector of the largest possible
                                   // values a solution can have --> upper bounds

        int dimension;// D

        std::set<std::vector<int>>
            init_solutions;// Matrix of initial solutions (pop_s * D)

        int stopping_criterion = 0;// ToDo ???
        int sim_counter = 0;       // ToDo ???

        std::vector<Arm> arms;// ToDo ???

        ////// PSO
        std::vector<std::multiset<MS_element, std::less<>>> MS_vec;
        std::vector<std::vector<Arm>> arms_vec;// Arm-Speicher eines jeden Paricles
        std::vector<Arm> current_particles;    // Arme der current iteration
        std::vector<std::vector<int>> velocity;// velocity
        std::vector<LUT> lookuptree_vec;

        std::function<double(std::vector<int>)> opti_func;

        std::vector<Arm> arms_global;
        LUT lookuptree_global;
        std::multiset<MS_element, std::less<>> MS_global;
        ////// PSO

       public:
        std::vector<solution> best_solutions;
        std::vector<std::vector<double>> diversity;

        int128_t calc_solution_code(std::vector<int> x);
        int run();

        void save_solution(int z);

        void PSO(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms);
        void MAB(int save_solution_every_x, std::vector<int> best_individual_arm_indices, int z);

        PSOMAB(std::function<double(std::vector<int>)> func, unsigned long max_gen, int pop_s,
               int stopping_criterion, unsigned seed, std::vector<int> s_ll,
               std::vector<int> s_ul, int D);
};

#endif// _PSOMAB_H_
