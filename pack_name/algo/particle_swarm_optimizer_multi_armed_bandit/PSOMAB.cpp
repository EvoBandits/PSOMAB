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

void PSOMAB::update_global_state(int arm_index_global, double r_before_update, double r_after_update) {
        auto it_global = global_sats.find(MS_element(arm_index_global, global_arms.at(arm_index_global).mean_reward()));
        int var_global = (*it_global).arm_index;

        // Falls zwei Lösungen den selben Mean Value haben, kann prinzipiell var!=arm_index auftreten. Dann muss im Baum
        // weiter iteriert werden bis var==arm_index um wirklich die richtige Lösung zu ziehen.
        while (var_global != arm_index_global) {
                it_global++;
                var_global = (*it_global).arm_index;
        }

        // lösche knoten zugehörig zu arm_index_global in MS_global (er wird
        // gezogen und verändert sich. für das update muss man ihn deshalb
        // löschen)
        global_sats.erase(it_global);

        // erhöhe um 1
        // erhöhe bei arms_global[arm_index_global] k um eins, da der arm (wenige zeilen weiter oben) gezogen worden ist.
        global_arms[arm_index_global].update_num_pulls(1);

        // update bei arms_global[index_global] r um den aktuellen reward
        // (lässt sich als diff berechnen)
        global_arms[arm_index_global].update_reward(r_after_update - r_before_update);

        // füge neu zu MS_global hinzu
        global_sats.insert(MS_element(var_global, global_arms.at(var_global).mean_reward()));
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
                // alle arm_index >= 0 existieren, -1 falls arm noch nicht vorhanden
                // existiert bereits
                if (arm_index >= 0) {
                        if (arm_index != best_individual_arm_indices[particle_index]) {
                                // Suche im lokalen SAT des i-ten Partikel nach entsprechendem Knoten
                                auto sat_node = local_sats[particle_index].find(MS_element(arm_index, local_arms[particle_index].at(arm_index).mean_reward()));

                                int note_index = (*sat_node).arm_index;
                                // If two nodes have the same mean value, the case var!=arm_index max occur
                                // In this case, the tree must be iterated further until var==arm_index in order to actually find the correct (SAT) node
                                while (note_index != arm_index) {
                                        sat_node++;
                                        note_index = (*sat_node).arm_index;
                                }
                                // Delete Node from Local SAT
                                local_sats[particle_index].erase(sat_node);

                                // Store current mean in cache (before update)
                                double old_mean_reward = local_arms[particle_index].at(note_index).reward();

                                // pull_arm
                                local_arms[particle_index].at(note_index).pull();

                                // global
                                // Store current mean in cache (after update)
                                double new_mean_reward = local_arms[particle_index].at(note_index).reward();
                                // Suche entsprechenden Arm im global LUT arm_index_global=suche von arms_vec[i].at(var) den index im lookuptree_global
                                const int arm_index_global = global_lookup_tree.search(search_index);

                                // Update global SAT
                                update_global_state(arm_index_global, old_mean_reward, new_mean_reward);

                                // Füge gezogenen Arm dem Lokal SAT hinzu (er wurde zuvor aus dem lok. SAT entfernt).
                                local_sats[particle_index].insert(MS_element(note_index, local_arms[particle_index].at(note_index).mean_reward()));
                        }
                } else {
                        // existiert noch nicht
                        // 0 = cost info, eigentlich nicht notwendig
                        Arm new_arm(pso.opti_func(), pso.particles()[particle_index].get_action_vector(), 0);

                        // füge Arm dem lokalen Arm Gedächtnis des i-ten Partikel zu
                        local_arms[particle_index].push_back(new_arm);

                        // for global
                        double old_mean_reward = local_arms[particle_index].back().reward();

                        // neuen Arm ziehen
                        local_arms[particle_index].back().pull();

                        // global
                        double new_mean_reward = local_arms[particle_index].back().reward();
                        // berechne "unique integer" aka search
                        int128_t search_index_global = calc_solution_code(local_arms[particle_index].back().get_action_vector());

                        // arm_index_global=suche von arms_vec[i].at(var) den index im lookuptree_GLOBAL
                        const int arm_index_global = global_lookup_tree.search(search_index_global);

                        if (arm_index_global >= 0) {
                                // falls knoten bereits im GLOBAL LUT existiert

                                // Update global SAT
                                update_global_state(arm_index_global, old_mean_reward, new_mean_reward);

                        } else {
                                // falls knoten noch nicht im GLOBAL LUT existiert

                                // füge neuen knoten in lookuptree ein
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

int PSOMAB::sum_arm_k() {
        int sum = 0;
        for (int i = 0; i < global_arms.size(); i++) {
                sum += global_arms.at(i).num_pulls();
        }
        return sum;
}

void PSOMAB::optimize() {
        for (int z = 1; z <= max_sim; z++) {

                // After first iteration
                if (z == 1) {
                        // save the solution that is currently considered to be the best
                        save_solution();
                }

                std::vector<Arm> best_individual_arms;
                std::vector<int> best_individual_arm_indices;

                double best_global_Q;
                best_global_Q = 1000000000;
                int best_global_index;         // index des glabal best arm
                int best_global_particle_index;// Welches der insgesamt m Partel den global
                                               // best arm enthält

                for (int k = 0; k < pso.num_particle(); k++) {

                        auto it_PSO = local_sats[k].begin();

                        int arm_index = (*it_PSO).arm_index;// index des besten arms der aktuellen Iteration

                        best_individual_arms.push_back(local_arms[k].at(arm_index));// weise es den Armen der aktuellen Iteration zu
                        best_individual_arm_indices.push_back(arm_index);

                        // update global best
                        if ((*it_PSO).Q < best_global_Q) {
                                best_global_Q = (*it_PSO).Q;
                                best_global_particle_index = k;
                                best_global_index = (*it_PSO).arm_index;
                        }
                }

                pso.step(best_global_particle_index, best_global_index, best_individual_arms, local_arms);

                MAB(best_individual_arm_indices);

                // Die m besten Arme werden in jeder Iteration erneut gezogen um bessere
                // Sample Means zu erhalten. Das passiert in der folgenden For loop
                for (int i = 0; i < pso.num_particle(); i++) {
                        auto it = local_sats[i].find(MS_element(best_individual_arm_indices[i], local_arms[i].at(best_individual_arm_indices[i]).mean_reward()));
                        int var = (*it).arm_index;

                        // Falls zwei Lösungen den selben
                        // Mean Value haben,
                        // kann prinzipiell var!=arm_index auftreten. Dann muss im Baum weiter
                        // iteriert werden bis var==arm_index um wirklich die richtige Lösung zu
                        // ziehen.
                        while (var != best_individual_arm_indices[i]) {
                                it++;
                                var = (*it).arm_index;
                        }

                        local_sats[i].erase(it);

                        double r_before_update = local_arms[i].at(best_individual_arm_indices[i]).reward();// für global

                        local_arms[i].at(best_individual_arm_indices[i]).pull();

                        // global

                        double r_after_update = local_arms[i].at(best_individual_arm_indices[i]).reward();
                        int128_t search_index_global = calc_solution_code(local_arms[i].at(best_individual_arm_indices[i]).get_action_vector());
                        const int arm_index_global = global_lookup_tree.search(search_index_global);

                        auto it_global = global_sats.find(MS_element(arm_index_global, global_arms.at(arm_index_global).mean_reward()));
                        int var_global = (*it_global).arm_index;

                        // Falls zwei Lösungen den selben Mean Value haben, kann prinzipiell var!=arm_index auftreten.
                        // Dann muss im Baum weiter iteriert werden bis var==arm_index um wirklich die richtige Lösung zu ziehen.
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
                        global_arms[arm_index_global].update_reward(r_after_update - r_before_update);

                        // füge neu zu MS_global hinzu
                        global_sats.insert(MS_element(var_global, global_arms.at(var_global).mean_reward()));

                        // global ende

                        //////////////
                        if (sum_arm_k() % 100 == 0) {
                                save_solution();
                        }// save solution
                        if (sum_arm_k() == max_sim) {
                                return;
                        }
                        //////////////

                        local_sats[i].insert(MS_element(best_individual_arm_indices[i], local_arms[i].at(best_individual_arm_indices[i]).mean_reward()));
                }
        }
}

void PSOMAB::save_solution() {
        int max_number_pulls = std::numeric_limits<int>::min();

        // nochmal überprüfen mit ursprungscode
        for (const auto &arm : global_arms) {
                if (arm.num_pulls() > max_number_pulls) {
                        max_number_pulls = arm.num_pulls();
                }
        }

        double ucb_norm_min = std::numeric_limits<int>::max();
        double ucb_norm_max = std::numeric_limits<int>::min();

        for (auto it : global_sats) {
                int arm_index = it.arm_index;
                ucb_norm_min = std::min(ucb_norm_min, global_arms.at(arm_index).mean_reward());
                ucb_norm_max = std::max(ucb_norm_max, global_arms.at(arm_index).mean_reward());
                if (global_arms.at(arm_index).num_pulls() == max_number_pulls) {
                        break;
                }
        }

        int return_index = 0;
        double best_ucb_value = std::numeric_limits<int>::max();
        for (auto it : global_sats) {
                int arm_index = it.arm_index;
                if (ucb_norm_max == ucb_norm_min) {
                        return_index = arm_index;
                }
                double ucb = 1 - (ucb_norm_max - global_arms.at(arm_index).mean_reward()) / (ucb_norm_max - ucb_norm_min) + sqrt(2 * log(sum_arm_k()) / global_arms.at(arm_index).num_pulls());
                if (ucb < best_ucb_value) {
                        best_ucb_value = ucb;
                        return_index = arm_index;
                }
                if (global_arms.at(arm_index).num_pulls() == max_number_pulls) {
                        break;
                }
        }

        // no noise
        // TRUE VALUE EINFACH AUF EINEN BELIEBIGEN WERT SETZEN; FALLS SIMULATION ZU
        // RECHENINTENSIV IST UND EXAKTER WERT OHNEHIN NICHT BEKANNT/BESTIMMBAR
        double true_value =
            global_arms.at(best_arm_index).true_value();

        best_solutions.emplace_back(sum_arm_k(), global_arms.at(return_index).get_action_vector(), global_arms.at(return_index).num_pulls(), global_arms.at(return_index).mean_reward(), true_value);
}

PSOMAB::PSOMAB(std::function<double(Eigen::VectorXi, int)> func, unsigned long max_gen, int pop_s, unsigned seed, const Eigen::VectorXi &x_lb, const Eigen::VectorXi &x_ub, int D) : max_simulation{max_gen} {

        pso = PSO(pop_s, D, x_lb, x_ub, std::move(func));

        //The following procedure ensures that only unique solutions are generated in the first iteration.
        for (int i = 0; i < pso.num_particle(); i++) {

                // erzeuge für jeden Partikel einen (lokalen) LUT und Füge (lokalen) LUT dem Vektor aller lokalen LUTs hinzu
                LUT lookuptree_temp;
                local_lookup_trees.push_back(lookuptree_temp);

                // insert into Lookuptree (LUT)
                // Berechne "unique integer" bzw. search key/index
                int128_t search_index = calc_solution_code(pso.particles()[i].get_action_vector());
                // Füge einen neuen Knoten mit (search index, 0) dem lokalen LUT hinzu. "0" deshalb, da es der erste Knoten ist.
                local_lookup_trees.at(i).insert(0, search_index);

                // x_i

                // Vektor an Armen
                std::vector<Arm> arms_temp;
                // arms_vec[i]: Arm-Gedächtnis
                local_arms.push_back(arms_temp);
                // (Arm-Vektor) des i-ten Partikels
                // Füge Arm "arms.at(i)" dem Gedächtnis des i-ten Partikels hinzu
                local_arms[i].push_back(pso.particles()[i]);

                // Ziehe entsprechenden arm 0: Es gibt arms.size() speicher_listen, in jeder liste wird hier nur das 1. Element befüllt.
                local_arms[i].at(0).pull();

                // current sample mean
                double Q_PSO = local_arms[i].at(0).mean_reward();

                // erzeuge für jeden Partikel einen (lokalen) SAT
                std::multiset<MS_element, std::less<>> MS_temp;
                // Füge (lokalen) SAT dem Vektor aller lokalen SATs hinzu
                local_sats.emplace_back(MS_temp);
                // Füge einen neuen Knoten mit (Q:PSO:sample mean, 0) dem lokalen SAT hinzu. 0: Index des ersten Arms (in jeder der arms.size() Listen = diese sind partikelspezifisch)
                local_sats[i].insert(MS_element(0, Q_PSO));

                /// global
                // berechne "unique integer" aka search index
                int128_t search_index_global = calc_solution_code(local_arms[i].at(0).get_action_vector());
                // füge entsprechenden knoten in den GLOBALEN LUT
                global_lookup_tree.insert(i, search_index);
                // füge den arm (zugehörig zum Knoten) in das globale Arm Gedächtnis
                global_arms.push_back(local_arms[i].at(0));
                // füge einen entsprechenden Knoten in den GLOBALEN SAT ein (i: Index im globalen Arm Gedächtnis, Q_PSO: Sample Mean) hier i oben 0, da hier ->globaler <- Baum aufgebaut wird
                global_sats.insert(MS_element(i, Q_PSO));
        }
}
std::vector<solution> PSOMAB::getBest_solutions() {
        return best_solutions;
}
