#ifndef _PSOMAB_H_
#define _PSOMAB_H_
#include "../../objects/arm/Arm.h"
#include "../../objects/solution/Solution.h"
#include "../particle_swarm_optimizer/PSO.h"
#include "Eigen/Core"
#include <cmath>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>
#include "../../util/SolutionCodeCalculation.h"


class PSOMAB {
       private:
        // local SATs, arm_memory, LUTs
        std::vector<std::multimap<double, int>> local_sats;
        std::vector<std::vector<Arm>> local_arm_memories;
        std::vector<std::unordered_map<Eigen::VectorXi, int>> local_lookup_tables;

        // global SAT, arm_memory, LUT
        std::multimap<double, int> global_sat;
        std::vector<Arm> global_arm_memory;
        std::unordered_map<Eigen::VectorXi, int> global_lookup_table;

        PSO pso;

        std::vector<int> retrieve_best_solutions();
        static int get_arm_index(const Arm& particle, std::unordered_map<Eigen::VectorXi, int> &lookup_table);
        static void delete_sat_node(int arm_index, Arm &arm, std::multimap<double, int> &sat);
        void sample_and_update(int particle_index,  int best_individual_arm_index);
        int max_num_pulls();
        int find_best_ucb();
        void save_current_best_solution();
        void save_history();

       public:
        PSOMAB(std::function<double(Eigen::VectorXi, int)> func, int max_sim, int pop_s, const Eigen::VectorXi &s_ll, const Eigen::VectorXi &s_ul, int D, bool use_random_location_update=false, bool cap_velocity=true);
        void optimize();
        std::vector<solution> best_solutions();
};

#endif// _PSOMAB_H_
