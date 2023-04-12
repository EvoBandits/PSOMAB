#include "PSOMAB.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <utility>

std::vector<int> PSOMAB::retrieve_best_solutions() {
        std::vector<int> best_individual_arm_indices;

        double best_global_mean_reward;
        best_global_mean_reward = std::numeric_limits<double>::max();

        for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                auto best_mean_element = *local_sats[particle_index].begin();
                int arm_index = best_mean_element.second;

                pso.best_individual_arms()[particle_index] = local_arm_memories[particle_index][arm_index];
                best_individual_arm_indices.push_back(arm_index);

                // update global best
                if (best_mean_element.first < best_global_mean_reward) {
                        best_global_mean_reward = best_mean_element.first;
                        pso.best_particle_index() = particle_index;
                }
        }
        return best_individual_arm_indices;
}

int PSOMAB::get_arm_index(const Arm &particle, std::unordered_map<Eigen::VectorXi, int> &lookup_table) {
        auto arm_index = lookup_table.find(particle.get_action_vector());
        if (arm_index == lookup_table.end())
                return -1;
        else
                return (*arm_index).second;
}

void PSOMAB::delete_sat_node(int arm_index, Arm &arm, std::multimap<double, int> &sat) {
        auto sat_node = sat.find(arm.mean_reward());

        while ((*sat_node).second != arm_index)
                sat_node++;

        sat.erase(sat_node);
}

void PSOMAB::sample_and_update(int particle_index, int arm_index_local) {
        if (arm_index_local >= 0) {
                Arm &local_arm = local_arm_memories[particle_index][arm_index_local];
                int arm_index_global = get_arm_index(local_arm, global_lookup_table);
                Arm &global_arm = global_arm_memory[arm_index_global];

                delete_sat_node(arm_index_local, local_arm, local_sats[particle_index]);
                delete_sat_node(arm_index_global, global_arm, global_sat);

                double old_reward = local_arm.reward();
                local_arm.pull();
                pso.update_simulation_budget();
                double new_reward = local_arm.reward();

                global_arm.update_num_pulls(1);
                global_arm.update_reward(new_reward - old_reward);

                local_sats[particle_index].emplace(local_arm.mean_reward(), arm_index_local);
                global_sat.emplace(global_arm.mean_reward(), arm_index_global);
        } else {
                local_arm_memories[particle_index].push_back(pso.particles()[particle_index]);
                Arm &local_arm = local_arm_memories[particle_index].back();
                arm_index_local =  (int) local_arm_memories[particle_index].size() - 1;

                local_arm.pull();
                pso.update_simulation_budget();

                global_arm_memory.push_back(local_arm_memories[particle_index].back());
                Arm &global_arm = global_arm_memory.back();
                int arm_index_global = (int) global_arm_memory.size() - 1;

                local_lookup_tables[particle_index].emplace(local_arm.get_action_vector(),arm_index_local);
                global_lookup_table.emplace(local_arm.get_action_vector(), arm_index_global);

                local_sats[particle_index].emplace(local_arm.mean_reward(), arm_index_local);
                global_sat.emplace(global_arm.mean_reward(), arm_index_global);
        }
}

int PSOMAB::max_num_pulls() {
        int max_number_pulls = std::numeric_limits<int>::min();
        for (const auto &arm : global_arm_memory) {
                max_number_pulls = std::max(max_number_pulls, arm.num_pulls());
        }
        return max_number_pulls;
}

int PSOMAB::find_best_ucb() {
        // find min mean of non-dominated set
        int arm_index_ucb_norm_min = (*global_sat.begin()).second;
        double ucb_norm_min = global_arm_memory[arm_index_ucb_norm_min].mean_reward();

        // find max mean of non-dominated set
        int max_number_pulls = max_num_pulls();
        double ucb_norm_max = std::numeric_limits<double>::min();

        for (auto global_sat_node : global_sat) {
                int arm_index = global_sat_node.second;
                ucb_norm_max = std::max(ucb_norm_max, global_arm_memory[arm_index].mean_reward());

                // checks if we are still in the non dominated-set (current mean <= mean_max_pulls)
                if (global_arm_memory[arm_index].num_pulls() == max_number_pulls) {
                        break;
                }
        }

        // find the solution of non-dominated set with the lowest associated UCB value
        int best_arm_index = 0;
        double best_ucb_value = std::numeric_limits<double>::max();

        for (auto global_sat_node : global_sat) {
                int arm_index = global_sat_node.second;
                if (ucb_norm_max == ucb_norm_min) {
                        best_arm_index = arm_index;
                }

                // transform sample mean to interval [0,1]
                double transformed_sample_mean = (global_arm_memory[arm_index].mean_reward() - ucb_norm_min) / (ucb_norm_max - ucb_norm_min);
                double penalty_term = sqrt(2 * log(pso.simulations_used()) / global_arm_memory[arm_index].num_pulls());
                double ucb = transformed_sample_mean + penalty_term;

                // new best solution is found
                if (ucb < best_ucb_value) {
                        best_ucb_value = ucb;
                        best_arm_index = arm_index;
                }

                // checks if we are still in the non dominated-set (current mean <= mean_max_pulls)
                if (global_arm_memory[arm_index].num_pulls() == max_number_pulls) {
                        break;
                }
        }
        return best_arm_index;
}

void PSOMAB::save_current_best_solution() {
        int best_arm_index = find_best_ucb();
        Arm &best_arm = global_arm_memory[best_arm_index];

        double true_value = best_arm.true_value();
        double num_pulls_all = pso.simulations_used();
        Eigen::VectorXi best_solution = best_arm.get_action_vector();
        double num_pulls_best = best_arm.num_pulls();
        double mean_value = best_arm.mean_reward();

        pso.best_solutions().emplace_back(num_pulls_all, best_solution, num_pulls_best, mean_value, true_value);
}

void PSOMAB::save_history() {
        if (pso.simulations_used() % 100 == 0) {
                save_current_best_solution();
        }
}

PSOMAB::PSOMAB(std::function<double(Eigen::VectorXi, int)> func, int max_sim, int pop_s, const Eigen::VectorXi &x_lb, const Eigen::VectorXi &x_ub, int D, bool use_random_location_update, bool cap_velocity) : pso(pop_s, D, x_lb, x_ub, std::move(func), max_sim, use_random_location_update, cap_velocity) {
        for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                std::unordered_map<Eigen::VectorXi, int> local_lookup_table;
                local_lookup_tables.push_back(local_lookup_table);

                local_lookup_tables[particle_index].emplace(pso.particles()[particle_index].get_action_vector(), 0);
                global_lookup_table.emplace(pso.particles()[particle_index].get_action_vector(), particle_index);

                std::vector<Arm> local_arm_memory;
                local_arm_memories.push_back(local_arm_memory);
                local_arm_memories[particle_index].push_back(pso.particles()[particle_index]);
                Arm &local_arm = local_arm_memories[particle_index].back();

                local_arm.pull();
                pso.update_simulation_budget();

                global_arm_memory.push_back(local_arm);

                std::multimap<double, int> local_sat;
                local_sats.push_back(local_sat);
                local_sats[particle_index].emplace(local_arm.mean_reward(), 0);
                global_sat.emplace(local_arm.mean_reward(), particle_index);
        }
        save_current_best_solution();
}

void PSOMAB::optimize() {
        while (true) {
                std::vector<int> best_individual_arm_indices = retrieve_best_solutions();

                pso.update_positions();

                for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                        // sample for updated particle and update local and global memory
                        int arm_index_local = get_arm_index(pso.particles()[particle_index], local_lookup_tables[particle_index]);
                        sample_and_update(particle_index, arm_index_local);
                        pso.save_history();
                        if (pso.budget_reached())
                                return;

                        // sample for best individual solution and update local and global memory
                        arm_index_local = best_individual_arm_indices[particle_index];
                        sample_and_update(particle_index, arm_index_local);
                        save_history();
                        if (pso.budget_reached())
                                return;
                }
        }
}

std::vector<solution> PSOMAB::best_solutions() {
        return pso.best_solutions();
}
void PSOMAB::memory_to_csv(const std::string &filename) {
        //ToDo: sum of pulls not equal to 10.000
        for (const auto& arm: global_arm_memory){
                pso.memory.emplace(arm.get_action_vector(), arm.num_pulls());
        }

        pso.memory_to_csv(filename);
}
