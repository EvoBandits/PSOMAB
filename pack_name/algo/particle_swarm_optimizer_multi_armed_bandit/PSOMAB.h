#ifndef _PSOMAB_H_
#define _PSOMAB_H_
#include "../../objects/arm/Arm.h"
#include "../../objects/look_up_tree/LUT.h"
#include "../../objects/solution/Solution.h"
#include "../particle_swarm_optimizer/PSO.h"
#include "Eigen/Core"
#include <boost/multiprecision/cpp_int.hpp>
#include <cmath>
#include <queue>
#include <random>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

struct MS_element {
        int arm_index;
        double Q;

        bool operator<(const MS_element &rhs) const { return Q < rhs.Q; }
        bool operator>(const MS_element &rhs) const { return Q > rhs.Q; }

        MS_element(int arm_index, double Q) : arm_index(arm_index), Q(Q) {}
};

class PSOMAB {
       private:
        unsigned long max_sim;// max number of simulations
        std::vector<solution> best_solutions;

        std::vector<std::multiset<MS_element, std::less<>>> local_sats;
        std::vector<std::vector<Arm>> local_arms;// Arm-Speicher eines jeden Paricles
        std::vector<LUT> local_lookup_trees;

        std::vector<Arm> global_arms;
        LUT global_lookup_tree;
        std::multiset<MS_element, std::less<>> global_sats;

        PSO pso;

        int128_t calc_solution_code(Eigen::VectorXi x);
        void MAB(std::vector<int> best_individual_arm_indices);
        void save_solution();
        int sum_arm_k();
        void update_global_state(int arm_index_global, double r_before_update, double r_after_update);
        void add_to_global_memory(int128_t search_index_global, const Arm &test);

       public:
        void optimize();
        std::vector<solution> getBest_solutions();
        PSOMAB(std::function<double(Eigen::VectorXi)> func, unsigned long max_gen, int pop_s, unsigned seed, const Eigen::VectorXi &s_ll, const Eigen::VectorXi &s_ul, int D);
};

#endif// _PSOMAB_H_
