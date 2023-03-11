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

// calculates the solution code, i.e. unique integer
int128_t PSOMAB::calc_solution_code(Eigen::VectorXi x) {
        int128_t search_index = 0;
        for (int i = 0; i < pso.dimension(); i++) {
                int exp = ceil(log10((pso.x_max()[i] - pso.x_min()[i]) + 1));
                search_index = search_index + (int128_t) (pow((pow(10, exp)), (pso.dimension() - 1) - i) * (x[i] - pso.x_min()[i]));
        }
        return search_index;
}

void PSOMAB::update_global_state(int arm_index_global, double old_reward, double new_reward) {
        auto global_sat_node = global_sats.find(MS_element(arm_index_global, global_arms.at(arm_index_global).mean_reward()));
        int global_note_index = (*global_sat_node).arm_index;

        // Falls zwei Lösungen denselben Mean Value haben, kann prinzipiell global_note_index!=arm_index auftreten. Dann muss im Baum
        // weiter iteriert werden bis global_note_index==arm_index, um wirklich die richtige Lösung zu ziehen.
        while (global_note_index != arm_index_global) {
                global_sat_node++;
                global_note_index = (*global_sat_node).arm_index;
        }

        // lösche knoten zugehörig zu arm_index_global in MS_global (er wird
        // gezogen und verändert sich. für das Update muss man ihn deshalb
        // löschen)
        global_sats.erase(global_sat_node);

        // erhöhe um 1
        // erhöhe bei arms_global[arm_index_global] k um eins, da der arm (wenige zeilen weiter oben) gezogen worden ist.
        global_arms[arm_index_global].update_num_pulls(1);

        // update bei arms_global[index_global] r um den aktuellen reward
        // (lässt sich als diff berechnen)
        global_arms[arm_index_global].update_reward(new_reward - old_reward);

        // füge neu zu MS_global hinzu
        global_sats.insert(MS_element(global_note_index, global_arms.at(global_note_index).mean_reward()));
}

void PSOMAB::add_to_global_memory(int128_t search_index_global, const Arm &test) {
        // füge den arm zu global arms hinzu
        global_arms.push_back(test);

        // füge neuen knoten in lookuptree ein
        int new_index_global = (int) global_arms.size() - 1;
        global_lookup_tree.insert(new_index_global, std::move(search_index_global));

        // füge neuen knoten in MS_GLOBAL ein
        global_sats.insert(MS_element(new_index_global, global_arms.at(new_index_global).mean_reward()));
}

void PSOMAB::MAB(std::vector<int> best_individual_arm_indices) {
        for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                // berechne "unique integer" aka search index
                int128_t search_index = calc_solution_code(pso.particles()[particle_index].get_action_vector());

                // Suche im lokalen LUT des i-ten partikel nach arm_index
                const int arm_index = local_lookup_trees[particle_index].search(search_index);

                // alle arm_index >= 0 existieren, -1, falls arm noch nicht vorhanden
                // existiert bereits
                if (arm_index >= 0) {
                        // ToDo: why is the following if statement necessary?
                        if (arm_index != best_individual_arm_indices[particle_index]) {
                                // Suche im lokalen SAT des i-ten Partikel nach entsprechendem Knoten
                                // ToDo: simplify this
                                auto sat_node = local_sats[particle_index].find(MS_element(arm_index, local_arms[particle_index].at(arm_index).mean_reward()));

                                int note_index = (*sat_node).arm_index;
                                // If two nodes have the same mean value, the case note_index!=arm_index max occur
                                // In this case, the tree must be iterated further until note_index==arm_index in order to actually find the correct (SAT) node
                                while (note_index != arm_index) {
                                        sat_node++;
                                        note_index = (*sat_node).arm_index;
                                }
                                // Delete Node from Local SAT
                                local_sats[particle_index].erase(sat_node);

                                // Store current mean in cache (before update)
                                double old_reward = local_arms[particle_index].at(note_index).reward();

                                // pull_arm
                                local_arms[particle_index].at(note_index).pull();

                                // global
                                // Store current mean in cache (after update)
                                double new_reward = local_arms[particle_index].at(note_index).reward();
                                // Suche entsprechenden Arm im global LUT arm_index_global=suche von arms_vec[i].at(var) den index im lookuptree_global
                                const int arm_index_global = global_lookup_tree.search(search_index);

                                // Update global SAT
                                update_global_state(arm_index_global, old_reward, new_reward);

                                // Füge gezogenen Arm dem Lokal SAT hinzu (er wurde zuvor aus dem lok. SAT entfernt).
                                // ToDo: simplify this
                                local_sats[particle_index].insert(MS_element(note_index, local_arms[particle_index].at(note_index).mean_reward()));
                        }
                } else {
                        // existiert noch nicht
                        // ToDo: why new arm? why not just add pso.particles()[particle_index] to local arms?
                        Arm new_arm(pso.opti_func(), pso.particles()[particle_index].get_action_vector());

                        // füge Arm dem lokalen Arm Gedächtnis des i-ten Partikel zu
                        local_arms[particle_index].push_back(new_arm);

                        // for global
                        double old_reward = local_arms[particle_index].back().reward();

                        // neuen Arm ziehen
                        local_arms[particle_index].back().pull();

                        // global
                        double new_reward = local_arms[particle_index].back().reward();

                        // berechne "unique integer" aka search_index
                        int128_t search_index_global = calc_solution_code(local_arms[particle_index].back().get_action_vector());

                        // arm_index_global=suche von arms_vec[i].at(var) den index im lookuptree_GLOBAL
                        const int arm_index_global = global_lookup_tree.search(search_index_global);


                        if (arm_index_global >= 0) {
                                // falls knoten bereits im GLOBAL LUT existiert

                                // Update global SAT
                                update_global_state(arm_index_global, old_reward, new_reward);

                        } else {
                                // falls knoten noch nicht im GLOBAL LUT existiert

                                // füge neuen knoten in LUT und SAT ein
                                add_to_global_memory(search_index, local_arms[particle_index].back());
                        }

                        // In lokalen LUT einfügen
                        int new_index = (int) local_arms[particle_index].size() - 1;
                        // root = (*root).InsertIntoTree(root, search_index, new_index); // fill
                        // search tree ,  (arms.size()-1) ist index des letzten Elements
                        local_lookup_trees[particle_index].insert(new_index, search_index);

                        // In lokalen SAT einfügen
                        local_sats[particle_index].insert(MS_element(new_index, local_arms[particle_index].at(new_index).mean_reward()));
                }
        }
}

void PSOMAB::optimize() {
        // loop will be stopped if simulation budget is reached (checked before each new simulation) ToDo: check if true

        while(true) {
                // ToDo: get rid of this if possible or "combine" with pso.best_individual_arms()
                // ToDo: abstract into separate function, something like retrieve_best_solution
                std::vector<int> best_individual_arm_indices;

                double best_global_Q;
                best_global_Q = std::numeric_limits<double>::max();

                for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                        MS_element best_mean_element = *local_sats[particle_index].begin();

                        int arm_index = best_mean_element.arm_index;

                        pso.best_individual_arms()[particle_index] = local_arms[particle_index].at(arm_index);
                        best_individual_arm_indices.push_back(arm_index);

                        // update global best
                        if (best_mean_element.Q < best_global_Q) {
                                best_global_Q = best_mean_element.Q;
                                pso.best_particle_index() = particle_index;
                        }
                }

                pso.step();

                // split into separate functions: sample solution, update_local_memory, update_global_memory, resample_best, ...
                MAB(best_individual_arm_indices);

                // Die jeweils besten Arme aller Particles werden erneut gezogen, um bessere Sample Means zu erhalten
                for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {
                        auto sat_node = local_sats[particle_index].find(MS_element(best_individual_arm_indices[particle_index], local_arms[particle_index].at(best_individual_arm_indices[particle_index]).mean_reward()));
                        int node_index = (*sat_node).arm_index;

                        // Falls zwei Lösungen denselben Mean Value haben, kann prinzipiell node_index!=arm_index auftreten. Dann muss im Baum weiter
                        // iteriert werden bis node_index==arm_index, um wirklich die richtige Lösung zu ziehen.
                        while (node_index != best_individual_arm_indices[particle_index]) {
                                sat_node++;
                                node_index = (*sat_node).arm_index;
                        }

                        local_sats[particle_index].erase(sat_node);

                        double old_mean_reward = local_arms[particle_index].at(best_individual_arm_indices[particle_index]).reward();// für global

                        local_arms[particle_index].at(best_individual_arm_indices[particle_index]).pull();

                        // global

                        double new_mean_reward = local_arms[particle_index].at(best_individual_arm_indices[particle_index]).reward();
                        int128_t search_index_global = calc_solution_code(local_arms[particle_index].at(best_individual_arm_indices[particle_index]).get_action_vector());
                        const int arm_index_global = global_lookup_tree.search(search_index_global);

                        auto it_global = global_sats.find(MS_element(arm_index_global, global_arms.at(arm_index_global).mean_reward()));
                        int var_global = (*it_global).arm_index;

                        // Falls zwei Lösungen denselben Mean Value haben, kann prinzipiell node_index!=arm_index auftreten.
                        // Dann muss im Baum weiter iteriert werden bis node_index==arm_index um wirklich die richtige Lösung zu ziehen.
                        while (var_global != arm_index_global) {
                                it_global++;
                                var_global = (*it_global).arm_index;
                        }

                        // lösche knoten zugehörig zu arm_index_global in MS_global
                        global_sats.erase(it_global);

                        // erhöhe um 1
                        // erhöhe bei arms_global[arm_index_global] k um eins
                        global_arms[arm_index_global].update_num_pulls(1);

                        // update bei arms_global[index_global] r um den aktuellen reward (lässt
                        // sich als diff berechnen)
                        global_arms[arm_index_global].update_reward(new_mean_reward - old_mean_reward);

                        // füge neu zu MS_global hinzu
                        global_sats.insert(MS_element(var_global, global_arms.at(var_global).mean_reward()));

                        // global ende

                        //////////////
                        if (pso.sum_num_pulls(global_arms) % 100 == 0) {
                                save_current_best_solution();
                        }// save solution
                        if (pso.sum_num_pulls(global_arms) >= pso.max_simulation()) {
                                return;
                        }
                        //////////////

                        // ToDo: vereinfachen
                        local_sats[particle_index].insert(MS_element(best_individual_arm_indices[particle_index], local_arms[particle_index].at(best_individual_arm_indices[particle_index]).mean_reward()));
                }
        }
}

int PSOMAB::max_num_pulls() const {
        int max_number_pulls = std::numeric_limits<int>::min();
        for (const auto &arm : global_arms) {
                max_number_pulls = std::max(max_number_pulls, arm.num_pulls());
        }
        return max_number_pulls;
}

int PSOMAB::select_ucb() {
        double ucb_norm_min = std::numeric_limits<double>::max();
        double ucb_norm_max = std::numeric_limits<double>::min();
        int best_arm_index = 0;
        double best_ucb_value = std::numeric_limits<double>::max();

        // ToDo: nochmal überprüfen mit ursprungscode

        int max_number_pulls = max_num_pulls();

        // ToDo: nochmal überprüfen
        for (auto global_sat_node : global_sats) {
                int arm_index = global_sat_node.arm_index;

                // ToDo: aus for-Schleife raus, da eh direkt der erste Knoten im Baum?
                ucb_norm_min = std::min(ucb_norm_min, global_arms.at(arm_index).mean_reward());
                ucb_norm_max = std::max(ucb_norm_max, global_arms.at(arm_index).mean_reward());

                // checks if we are still in the non dominated-set (mean <= mean_max_pulls)
                if (global_arms.at(arm_index).num_pulls() == max_number_pulls) {
                        break;
                }
        }

        for (auto global_sat_node : global_sats) {
                int arm_index = global_sat_node.arm_index;
                if (ucb_norm_max == ucb_norm_min) {
                        best_arm_index = arm_index;
                }

                // transform sample mean to interval [0,1]
                double transformed_sample_mean = (global_arms.at(arm_index).mean_reward() - ucb_norm_min) / (ucb_norm_max - ucb_norm_min);
                double penalty_term = sqrt(2 * log(pso.sum_num_pulls(global_arms)) / global_arms.at(arm_index).num_pulls());
                double ucb = transformed_sample_mean + penalty_term;

                if (ucb < best_ucb_value) {
                        best_ucb_value = ucb;
                        best_arm_index = arm_index;
                }

                // checks if we are still in the non dominated-set (mean <= mean_max_pulls)
                if (global_arms.at(arm_index).num_pulls() == max_number_pulls) {
                        break;
                }
        }
        return best_arm_index;
}

void PSOMAB::save_current_best_solution() {
        int best_arm_index = select_ucb();

        double true_value = global_arms.at(best_arm_index).true_value();
        double num_pulls_all = pso.sum_num_pulls(global_arms);
        Eigen::VectorXi best_solution = global_arms.at(best_arm_index).get_action_vector();
        double num_pulls_best = global_arms.at(best_arm_index).num_pulls();
        double mean_value = global_arms.at(best_arm_index).mean_reward();

        pso.best_solutions().emplace_back(num_pulls_all, best_solution, num_pulls_best, mean_value, true_value);
}

PSOMAB::PSOMAB(std::function<double(Eigen::VectorXi, int)> func, int max_sim, int pop_s, unsigned seed, const Eigen::VectorXi &x_lb, const Eigen::VectorXi &x_ub, int D) : pso(pop_s, D, x_lb, x_ub, std::move(func), max_sim){

        //The following procedure ensures that only unique solutions are generated in the first iteration.
        for (int particle_index = 0; particle_index < pso.num_particle(); particle_index++) {

                // erzeuge für jedes Partikel einen (lokalen) LUT und füge (lokalen) LUT dem Vektor aller lokalen LUTs hinzu
                // ToDo: rename -> LUT not temporary, just variable
                LUT lookuptree_temp;

                // insert into Lookuptree (LUT)
                local_lookup_trees.push_back(lookuptree_temp);

                // Berechne "unique integer" bzw. search key/index
                int128_t search_index = calc_solution_code(pso.particles()[particle_index].get_action_vector());
                // Füge einen neuen Knoten mit (search index, 0) dem lokalen LUT hinzu. "0" deshalb, da es der erste Knoten ist.
                local_lookup_trees.at(particle_index).insert(0, search_index);

                // x_i

                // Vektor an Armen
                // ToDo: rename -> Arm not temporary, just variable
                std::vector<Arm> arms_temp;
                // arms_vec[i]: Arm-Gedächtnis
                local_arms.push_back(arms_temp);
                // (Arm-Vektor) des i-ten Partikels
                // Füge Arm "arms.at(i)" dem Gedächtnis des i-ten Partikels hinzu
                local_arms[particle_index].push_back(pso.particles()[particle_index]);

                // Ziehe entsprechenden arm 0: Es gibt arms.size() speicher_listen, in jeder liste wird hier nur das 1. Element befüllt.
                local_arms[particle_index].at(0).pull();

                // current sample mean
                // ToDo: replace Q with mean_reward (everywhere)
                double Q_PSO = local_arms[particle_index].at(0).mean_reward();

                // erzeuge für jeden Partikel einen (lokalen) SAT
                // ToDo: rename -> SAT not temporary, just variable
                std::multiset<MS_element, std::less<>> sat_temp;
                // Füge (lokalen) SAT dem Vektor aller lokalen SATs hinzu
                local_sats.emplace_back(sat_temp);
                // Füge einen neuen Knoten mit (Q:PSO:sample mean, 0) dem lokalen SAT hinzu. 0: Index des ersten Arms (in jeder der arms.size() Listen = diese sind partikelspezifisch)
                local_sats[particle_index].insert(MS_element(0, Q_PSO));

                /// global, no check for duplicates necessary as we start with unique solutions
                // füge entsprechenden knoten in den GLOBALEN LUT
                global_lookup_tree.insert(particle_index, search_index);
                // füge den arm (zugehörig zum Knoten) in das globale Arm Gedächtnis
                global_arms.push_back(local_arms[particle_index].at(0));
                // füge einen entsprechenden Knoten in den GLOBALEN SAT ein (i: Index im globalen Arm Gedächtnis, Q_PSO: Sample Mean) hier i oben 0, da hier ->globaler <- Baum aufgebaut wird
                global_sats.insert(MS_element(particle_index, Q_PSO));
        }
        // save the solution that is currently considered to be the best
        save_current_best_solution();
}
std::vector<solution> PSOMAB::best_solutions() {
        return pso.best_solutions();
}
