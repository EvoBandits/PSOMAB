#include "PSOMAB.h"
#include "../../objects/arm/Arm.h"
#include "../../util/RandomNumber.h"
#include <algorithm>// std::sort
#include <boost/multiprecision/cpp_int.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>
#include <random>
#include <set>
#include <utility>

// std::uniform_int_distribution<int> uniform_int_distribution(a, b);
// std::uniform_real_distribution<double> uniform_real_distribution(a, b);

// ToDo: auslagern der funktion da sie nichts mit PSOMAB explizit zu tun hat
// calculates the solution code, i.e. unique integer
int128_t PSOMAB::calc_solution_code(Eigen::VectorXi action_vector) {
        int128_t search_index = 0;
        for (int i = 0; i < pso.dimension(); i++) {
                int exp = ceil(log10((pso.x_max()[i] - pso.x_min()[i]) + 1));
                search_index = search_index + (int128_t) (pow((pow(10, exp)), (pso.dimension() - 1) - i) * (action_vector[i] - pso.x_min()[i]));
        }
        return search_index;
}

void PSOMAB::update_global_state(int arm_index_global, double old_reward, double new_reward) {
        delete_sat_node(arm_index_global, global_arm_memory.at(arm_index_global), global_sat);

        global_arm_memory[arm_index_global].update_num_pulls(1);
        global_arm_memory[arm_index_global].update_reward(new_reward - old_reward);

        global_sat.insert(MS_element(arm_index_global, global_arm_memory.at(arm_index_global).mean_reward()));
}

void PSOMAB::add_to_global_memory(const Arm &particle) {
        int128_t search_index = calc_solution_code(particle.get_action_vector());
        global_arm_memory.push_back(particle);

        // insert arm into global lookup tree
        int new_index_global = (int) global_arm_memory.size() - 1;
        global_lookup_tree.insert(new_index_global, search_index);

        // insert new element into SAT
        double mean_reward = global_arm_memory.at(new_index_global).mean_reward();
        MS_element new_element_global(new_index_global, mean_reward);
        global_sat.insert(new_element_global);
}

std::vector<int> PSOMAB::retrieve_best_solutions() {
        // ToDo: get rid of this if possible or "combine" with pso.best_individual_arms()
        std::vector<int> best_individual_arm_indices;

        double best_global_mean_reward;
        best_global_mean_reward = std::numeric_limits<double>::max();

        for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                MS_element best_mean_element = *local_sats[particle_index].begin();

                int arm_index = best_mean_element.arm_index;

                pso.best_individual_arms()[particle_index] = local_arm_memories[particle_index].at(arm_index);
                best_individual_arm_indices.push_back(arm_index);

                // update global best
                if (best_mean_element.mean_reward < best_global_mean_reward) {
                        best_global_mean_reward = best_mean_element.mean_reward;
                        pso.best_particle_index() = particle_index;
                }
        }
        return best_individual_arm_indices;
}

int PSOMAB::get_arm_index(const Arm &particle, LUT &local_lookup_tree) {
        int128_t search_index = calc_solution_code(particle.get_action_vector());

        const int arm_index = local_lookup_tree.search(search_index);

        return arm_index;
}

void PSOMAB::delete_sat_node(int arm_index, Arm &arm, std::multiset<MS_element, std::less<>> &sat){
        double existing_arm_mean_reward = arm.mean_reward();

        MS_element existing_arm = MS_element(arm_index, existing_arm_mean_reward);
        auto sat_node = sat.find(existing_arm);

        int sat_node_arm_index = (*sat_node).arm_index;

        // If two nodes have the same mean value, the case sat_node_arm_index!=arm_index max occur
        // In this case, the tree must be iterated further until sat_node_arm_index==arm_index in order to actually find the correct (SAT) node
        while (sat_node_arm_index != arm_index) {
                sat_node++;
                sat_node_arm_index = (*sat_node).arm_index;
        }

        sat.erase(sat_node);
}

// ToDo: sample und resample haben viele gleiche code zeilen, kann man vlt noch weiter abstrahieren
void PSOMAB::sample_and_update(int particle_index, int best_individual_arm_index) {
        int arm_index_local = get_arm_index(pso.particles()[particle_index], local_lookup_trees[particle_index]);

        if (arm_index_local >= 0) {
                bool skip_sample = (arm_index_local == best_individual_arm_index) && prevent_from_sampling_twice_;

                if (!skip_sample) {
                        // search local SAT for corresponding node and erase node
                        delete_sat_node(arm_index_local, local_arm_memories[particle_index].at(arm_index_local), local_sats[particle_index]);

                        // save old reward, pull, save new reward
                        double old_reward = local_arm_memories[particle_index].at(arm_index_local).reward();
                        local_arm_memories[particle_index].at(arm_index_local).pull();
                        double new_reward = local_arm_memories[particle_index].at(arm_index_local).reward();

                        // global update:
                        const int arm_index_global = get_arm_index(pso.particles()[particle_index], global_lookup_tree);
                        update_global_state(arm_index_global, old_reward, new_reward);

                        // Update local SAT
                        double pulled_arm_mean_reward = local_arm_memories[particle_index].at(arm_index_local).mean_reward();
                        MS_element pulled_arm = MS_element(arm_index_local, pulled_arm_mean_reward);
                        local_sats[particle_index].insert(pulled_arm);
                }
        } else {
                // add new arm to local memory of particle
                local_arm_memories[particle_index].push_back(pso.particles()[particle_index]);

                // save old reward (here always 0), pull, save new reward
                double old_reward = local_arm_memories[particle_index].back().reward();
                local_arm_memories[particle_index].back().pull();
                double new_reward = local_arm_memories[particle_index].back().reward();


                // Update global SAT
                const int arm_index_global = get_arm_index(pso.particles()[particle_index], local_lookup_trees[particle_index]);
                if (arm_index_global >= 0)
                        update_global_state(arm_index_global, old_reward, new_reward);
                else
                        add_to_global_memory(local_arm_memories[particle_index].back());

                // Insert into local lookup tree
                int new_index = (int) local_arm_memories[particle_index].size() - 1;
                int128_t search_index = calc_solution_code(pso.particles()[particle_index].get_action_vector());
                local_lookup_trees[particle_index].insert(new_index, search_index);

                // Update local SAT
                double pulled_arm_mean_reward = local_arm_memories[particle_index].at(new_index).mean_reward();
                MS_element pulled_arm = MS_element(new_index, pulled_arm_mean_reward);
                local_sats[particle_index].insert(pulled_arm);
        }
}

void PSOMAB::resample_and_update(int particle_index, int best_individual_arm_index){
        // search local SAT for corresponding node and erase node
        delete_sat_node(best_individual_arm_index, local_arm_memories[particle_index].at(best_individual_arm_index), local_sats[particle_index]);

        // save old reward, pull, save new reward
        double old_reward = local_arm_memories[particle_index].at(best_individual_arm_index).reward();
        local_arm_memories[particle_index].at(best_individual_arm_index).pull();
        double new_reward = local_arm_memories[particle_index].at(best_individual_arm_index).reward();


        // global update:
        const int arm_index_global = get_arm_index(local_arm_memories[particle_index].at(best_individual_arm_index), global_lookup_tree);
        update_global_state(arm_index_global, old_reward, new_reward);

        // Update local SAT
        double best_individual_mean_reward = local_arm_memories[particle_index].at(best_individual_arm_index).mean_reward();
        MS_element best_individual_arm = MS_element(best_individual_arm_index, best_individual_mean_reward);
        local_sats[particle_index].insert(best_individual_arm);
}

bool PSOMAB::budget_reached(){
        return (pso.sum_num_pulls(global_arm_memory) >= pso.max_simulation());
}

void PSOMAB::save_history(){
        if (pso.sum_num_pulls(global_arm_memory) % 100 == 0) {
                save_current_best_solution();
        }
}

void PSOMAB::optimize() {
        while(true) {
                // find local bast solutions
                std::vector<int> best_individual_arm_indices = retrieve_best_solutions();

                // update particle positions
                pso.step(0);

                for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                        // sample for updated particle and update local and global memory
                        sample_and_update(particle_index, best_individual_arm_indices[particle_index]);
                        save_history();
                        if(budget_reached()) return ;

                        // resample best solution and update local and global memory
                        resample_and_update(particle_index, best_individual_arm_indices[particle_index]);
                        save_history();
                        if(budget_reached()) return ;
                }
        }
}

int PSOMAB::max_num_pulls() const {
        int max_number_pulls = std::numeric_limits<int>::min();
        for (const auto &arm : global_arm_memory) {
                max_number_pulls = std::max(max_number_pulls, arm.num_pulls());
        }
        return max_number_pulls;
}

int PSOMAB::find_best_ucb() {
        // find min mean of non-dominated set
        int arm_index_ucb_norm_min = (*global_sat.begin()).arm_index;
        double ucb_norm_min = global_arm_memory.at(arm_index_ucb_norm_min).mean_reward();

        // find max mean of non-dominated set
        int max_number_pulls = max_num_pulls();
        double ucb_norm_max = std::numeric_limits<double>::min();

        for (auto global_sat_node : global_sat) {
                int arm_index = global_sat_node.arm_index;
                ucb_norm_max = std::max(ucb_norm_max, global_arm_memory.at(arm_index).mean_reward());

                // checks if we are still in the non dominated-set (current mean <= mean_max_pulls)
                if (global_arm_memory.at(arm_index).num_pulls() == max_number_pulls) {
                        break;
                }
        }

        // find the solution of non-dominated set with the lowest associated UCB value
        int best_arm_index = 0;
        double best_ucb_value = std::numeric_limits<double>::max();

        for (auto global_sat_node : global_sat) {
                int arm_index = global_sat_node.arm_index;
                if (ucb_norm_max == ucb_norm_min) {
                        best_arm_index = arm_index;
                }

                // transform sample mean to interval [0,1]
                double transformed_sample_mean = (global_arm_memory.at(arm_index).mean_reward() - ucb_norm_min) / (ucb_norm_max - ucb_norm_min);
                double penalty_term = sqrt(2 * log(pso.sum_num_pulls(global_arm_memory)) / global_arm_memory.at(arm_index).num_pulls());
                double ucb = transformed_sample_mean + penalty_term;

                // new best solution is found
                if (ucb < best_ucb_value) {
                        best_ucb_value = ucb;
                        best_arm_index = arm_index;
                }

                // checks if we are still in the non dominated-set (current mean <= mean_max_pulls)
                if (global_arm_memory.at(arm_index).num_pulls() == max_number_pulls) {
                        break;
                }
        }
        return best_arm_index;
}

void PSOMAB::save_current_best_solution() {
        int best_arm_index = find_best_ucb();

        double true_value = global_arm_memory.at(best_arm_index).true_value();
        double num_pulls_all = pso.sum_num_pulls(global_arm_memory);
        Eigen::VectorXi best_solution = global_arm_memory.at(best_arm_index).get_action_vector();
        double num_pulls_best = global_arm_memory.at(best_arm_index).num_pulls();
        double mean_value = global_arm_memory.at(best_arm_index).mean_reward();

        pso.best_solutions().emplace_back(num_pulls_all, best_solution, num_pulls_best, mean_value, true_value);
}

PSOMAB::PSOMAB(std::function<double(Eigen::VectorXi, int)> func, int max_sim, int pop_s, unsigned seed, const Eigen::VectorXi &x_lb, const Eigen::VectorXi &x_ub, int D, bool use_random_location_update, bool prevent_from_sampling_twice) : pso(pop_s, D, x_lb, x_ub, std::move(func), max_sim, prevent_from_sampling_twice){
        prevent_from_sampling_twice_ = prevent_from_sampling_twice;
        for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                // create local lookup tree
                LUT local_lookup_tree;
                local_lookup_trees.push_back(local_lookup_tree);

                int128_t search_index = calc_solution_code(pso.particles()[particle_index].get_action_vector());
                local_lookup_trees.at(particle_index).insert(0, search_index);

                // create local arm memory
                std::vector<Arm> local_arm_memory;
                local_arm_memories.push_back(local_arm_memory);
                local_arm_memories[particle_index].push_back(pso.particles()[particle_index]);

                // pull the current particle
                local_arm_memories[particle_index].at(0).pull();
                double mean_reward = local_arm_memories[particle_index].at(0).mean_reward();

                // create local SAT
                std::multiset<MS_element, std::less<>> local_sat;
                local_sats.push_back(local_sat);
                local_sats[particle_index].insert(MS_element(0, mean_reward));

                //add particle to global lookup tree, global arm memory and global SAT (index of particle in arm memory = particle index)
                global_lookup_tree.insert(particle_index, search_index);
                global_arm_memory.push_back(local_arm_memories[particle_index].at(0));
                global_sat.insert(MS_element(particle_index, mean_reward));
        }
        save_current_best_solution();
}
std::vector<solution> PSOMAB::best_solutions() {
        return pso.best_solutions();
}
