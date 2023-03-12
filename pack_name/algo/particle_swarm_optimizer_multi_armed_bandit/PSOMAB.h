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
        double mean_reward; // key
        int arm_index;

        bool operator<(const MS_element &rhs) const { return mean_reward < rhs.mean_reward; }
        bool operator>(const MS_element &rhs) const { return mean_reward > rhs.mean_reward; }

        MS_element(int arm_index, double mean_reward) : arm_index(arm_index), mean_reward(mean_reward) {}
};

class PSOMAB {
       private:
        // local SATs, arm_memory, LUTs
        std::vector<std::multiset<MS_element, std::less<>>> local_sats;
        std::vector<std::vector<Arm>> local_arm_memories;
        std::vector<LUT> local_lookup_trees;

        // global SAT, arm_memory, LUT
        std::multiset<MS_element, std::less<>> global_sat;
        std::vector<Arm> global_arm_memory;
        LUT global_lookup_tree;

        PSO pso;

        int128_t calc_solution_code(Eigen::VectorXi x);
        void MAB(std::vector<int> best_individual_arm_indices);
        void save_current_best_solution();
        void update_global_state(int arm_index_global, double old_reward, double new_reward);
        void add_to_global_memory(int128_t search_index_global, const Arm &test);
        int find_best_ucb();

       public:
        void optimize();
        PSOMAB(std::function<double(Eigen::VectorXi, int)> func, int max_sim, int pop_s, unsigned seed, const Eigen::VectorXi &s_ll, const Eigen::VectorXi &s_ul, int D);
        std::vector<solution> best_solutions();
        int max_num_pulls() const;
};

#endif// _PSOMAB_H_
