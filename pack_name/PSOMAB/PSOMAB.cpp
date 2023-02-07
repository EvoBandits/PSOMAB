#include "PSOMAB.h"
#include "Arm.h"
#include "RandomNumber.h"
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
        for (int i = 0; i < dimension; i++) {
                int exp = ceil(log10((vec_x_max[i] - vec_x_min[i]) + 1));
                search_index = search_index + (int128_t) (pow((pow(10, exp)), (dimension - 1) - i) * (x[i] - vec_x_min[i]));
        }
        return search_index;
}

//
void PSOMAB::PSO(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms) {
        for (int k = 0; k < m; k++) {

                double c1 = 2.5;
                double c2 = 1;
                double v = 0.2;

                // with eigen for loop can bis discarded
                for (int g = 0; g < dimension; g++) {
                        // ToDo: ist runden hier richtig?
                        std::uniform_real_distribution<double> uniform_real_distribution_c1(0, c1);
                        std::uniform_real_distribution<double> uniform_real_distribution_c2(0, c2);
                        int b1 = round(uniform_real_distribution_c1(generator) * 1.0 * (best_individual_arms[k].get_action_vector()[g] - current_particles[k].get_action_vector()[g]));
                        int b2 = round(uniform_real_distribution_c2(generator) * 1.0 * (arms_vec[best_global_particle_index][best_global_index].get_action_vector()[g] - current_particles[k].get_action_vector()[g]));

                        velocity[k][g] = round(v * velocity[k][g] + b1 + b2);

                        int new_value = current_particles[k].get_action_vector()[g] + velocity[k][g];

                        // ToDo: ist das das richtige vorgehen bei werten außerhalb der range?
                        if (new_value > vec_x_max[g] || new_value < vec_x_min[g]) {
                                std::uniform_int_distribution<int> uniform_int_distribution(vec_x_min[g], vec_x_max[g]);
                                new_value = uniform_int_distribution(generator);
                        }
                        current_particles[k].set_action_vector_element(g, new_value);
                }
        }
}

void PSOMAB::MAB(std::vector<int> best_individual_arm_indices) {
        // current_particles.size() == m
        for (int i = 0; i < current_particles.size(); i++) {
                // berechne "unique integer" aka search index
                int128_t search_index = calc_solution_code(current_particles[i].get_action_vector());
                // Suche im lokalen LUT des i-ten partikel nach arm_index
                const int arm_index = lookuptree_vec[i].search(search_index);
                // alle arm_index >= 0 existieren, -1 falls arm noch nicht vorhanden
                // existiert bereits
                if (arm_index >= 0) {
                        if (arm_index != best_individual_arm_indices[i]) {
                                // Suche im lokalen SAT des i-ten Partikel nach entsprechendem Knoten
                                auto it = MS_vec[i].find(MS_element(arm_index, arms_vec[i].at(arm_index).get_r() / arms_vec[i].at(arm_index).get_k()));

                                int var = (*it).arm_index;
                                // If two nodes have the same mean value, the case var!=arm_index max occur
                                // In this case, the tree must be iterated further until var==arm_index in order to actually find the correct (SAT) node
                                while (var != arm_index) {
                                        it++;
                                        var = (*it).arm_index;
                                }
                                // Delete Node from Local SAT
                                MS_vec[i].erase(it);

                                // Store current mean in cache (before update)
                                double r_before_update = arms_vec[i].at(var).get_r();

                                // pull_arm
                                arms_vec[i].at(var).pull_arm();

                                // global
                                // Store current mean in cache (after update)
                                double r_after_update = arms_vec[i].at(var).get_r();
                                // Suche entsprechenden Arm im global LUT arm_index_global=suche von arms_vec[i].at(var) den index im lookuptree_global
                                const int arm_index_global = lookuptree_global.search(search_index);
                                auto it_global = MS_global.find(MS_element(arm_index_global, arms_global.at(arm_index_global).get_r() / arms_global.at(arm_index_global).get_k()));
                                // Suche im GLOBALEN SAT nach entsprechendem Knoten
                                int var_global = (*it_global).arm_index;
                                // If two nodes have the same mean value, the case var!=arm_index max occur
                                //In this case, the tree must be iterated further until var==arm_index in order to actually find the correct (SAT) node
                                while (var_global != arm_index_global) {
                                        it_global++;
                                        var_global = (*it_global).arm_index;
                                }
                                // lösche knoten zugehörig zu arm_index_global in MS_global
                                MS_global.erase(it_global);

                                // erhöhe um 1 // IM Globalen Arm Gedächtnis muss der sim
                                // counter um 1 erhöht werden, da der arm (wenige zeilen
                                // weiter oben) gezogen worden ist.
                                // erhöhe bei arms_global[arm_index_global] k um eins
                                arms_global[arm_index_global].update_k(1);

                                // Im Globalen Arm Gedächtnis muss der Sample mean geupdatet werden, da (wenige zeilen weiter oben) der Arm gezogen worden ist.
                                // update bei arms_global[index_global] r um den aktuellen reward (lässt sich als diff berechnen)
                                arms_global[arm_index_global].update_r(r_after_update - r_before_update);

                                // füge neu zu MS_global hinzu
                                MS_global.insert(MS_element(var_global, arms_global.at(var_global).get_r() / arms_global.at(var_global).get_k()));
                                //////end global////////

                                if (sum_arm_k() % 100 == 0) {
                                        save_solution();
                                }// save solution
                                if (sum_arm_k() == max_sim) {
                                        return;
                                }
                                //////////////

                                MS_vec[i].insert(MS_element(
                                    var,
                                    arms_vec[i].at(var).get_r() / arms_vec[i].at(var).get_k()));// Füge gezogenen Arm dem Lokal
                                                                                                // SAT hinzu (er wurde zuvor
                                                                                                // aus dem lok. SAT entfernt).
                        }
                } else {
                        // existiert noch nicht
                        // 0 = cost info, eigentlich nicht notwendig
                        Arm new_arm(opti_func, current_particles[i].get_action_vector(), 0);

                        // füge Arm dem lokalen Arm Gedächtnis des i-ten Partikel zu
                        arms_vec[i].push_back(new_arm);

                        // for global
                        double r_before_update = arms_vec[i].back().get_r();

                        // neuen Arm ziehen
                        arms_vec[i].back().pull_arm();

                        // global
                        double r_after_update = arms_vec[i].back().get_r();
                        // berechne "unique integer" aka search
                        int128_t search_index_global = calc_solution_code(arms_vec[i].back().get_action_vector());

                        // arm_index_global=suche von arms_vec[i].at(var) den index im lookuptree_GLOBAL
                        const int arm_index_global = lookuptree_global.search(search_index_global);

                        // falls knoten bereits im GLOBAL LUT existiert
                        if (arm_index_global >= 0) {
                                auto it_global = MS_global.find(MS_element(arm_index_global, arms_global.at(arm_index_global).get_r() / arms_global.at(arm_index_global).get_k()));
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
                                MS_global.erase(it_global);

                                // erhöhe um 1
                                // erhöhe bei arms_global[arm_index_global] k um eins
                                arms_global[arm_index_global].update_k(1);

                                // update bei arms_global[index_global] r um den aktuellen reward
                                // (lässt sich als diff berechnen)
                                arms_global[arm_index_global].update_r(r_after_update - r_before_update);

                                // füge neu zu MS_global hinzu
                                MS_global.insert(MS_element(var_global, arms_global.at(var_global).get_r() / arms_global.at(var_global).get_k()));


                        } else {
                                // existiert im GLOBAL LUT noch nicht

                                // füge den arm zu global arms hinzu
                                arms_global.push_back(arms_vec[i].back());

                                // füge neuen knoten in lookuptree ein
                                int new_index_global = (int) arms_global.size() - 1;
                                lookuptree_global.insert(new_index_global, search_index);

                                // füge neuen knoten in MS_GLOBAL ein
                                MS_global.insert(MS_element(new_index_global, arms_global.at(new_index_global).get_r() / arms_global.at(new_index_global).get_k()));

                        }
                        // global ende

                        //////////////
                        if (sum_arm_k() % 100 == 0) {
                                save_solution();
                        }// save solution
                        if (sum_arm_k() == max_sim) {
                                return;
                        }
                        //////////////

                        // In lokalen LUT einfügen
                        int new_index = (int) arms_vec[i].size() - 1;
                        // root = (*root).InsertIntoTree(root, search_index, new_index); // fill
                        // search tree ,  (arms.size()-1) ist index des letzten Elements
                        lookuptree_vec[i].insert(new_index, search_index);

                        // In lokalen SAT einfügen
                        MS_vec[i].insert(MS_element(new_index, arms_vec[i].at(new_index).get_r() / arms_vec[i].at(new_index).get_k()));
                }
        }
}

int PSOMAB::sum_arm_k() {
        int sum = 0;
        for (int i = 0; i < arms_global.size(); i++) {
                sum += arms_global.at(i).get_k();
        }
        return sum;
}

void PSOMAB::run() {
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

                for (int k = 0; k < m; k++) {

                        auto it_PSO = MS_vec[k].begin();

                        int arm_index = (*it_PSO).arm_index;// index des besten arms der aktuellen Iteration

                        best_individual_arms.push_back(arms_vec[k].at(arm_index));// weise es den Armen der aktuellen Iteration zu
                        best_individual_arm_indices.push_back(arm_index);

                        // update global best
                        if ((*it_PSO).Q < best_global_Q) {
                                best_global_Q = (*it_PSO).Q;
                                best_global_particle_index = k;
                                best_global_index = (*it_PSO).arm_index;
                        }
                }

                PSOMAB::PSO(best_global_particle_index, best_global_index, best_individual_arms);

                MAB(best_individual_arm_indices);

                // Die m besten Arme werden in jeder Iteration erneut gezogen um bessere
                // Sample Means zu erhalten. Das passiert in der folgenden For loop
                for (int i = 0; i < current_particles.size(); i++) {
                        auto it = MS_vec[i].find(MS_element(best_individual_arm_indices[i], arms_vec[i].at(best_individual_arm_indices[i]).get_r() / arms_vec[i].at(best_individual_arm_indices[i]).get_k()));
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

                        MS_vec[i].erase(it);

                        double r_before_update = arms_vec[i].at(best_individual_arm_indices[i]).get_r();// für global

                        arms_vec[i].at(best_individual_arm_indices[i]).pull_arm();

                        // global

                        double r_after_update = arms_vec[i].at(best_individual_arm_indices[i]).get_r();
                        int128_t search_index_global = calc_solution_code(arms_vec[i].at(best_individual_arm_indices[i]).get_action_vector());
                        const int arm_index_global = lookuptree_global.search(search_index_global);

                        auto it_global = MS_global.find(MS_element(arm_index_global, arms_global.at(arm_index_global).get_r() / arms_global.at(arm_index_global).get_k()));
                        int var_global = (*it_global).arm_index;

                        // Falls zwei Lösungen den selben Mean Value haben, kann prinzipiell var!=arm_index auftreten.
                        // Dann muss im Baum weiter iteriert werden bis var==arm_index um wirklich die richtige Lösung zu ziehen.
                        while (var_global != arm_index_global) {
                                it_global++;
                                var_global = (*it_global).arm_index;
                        }

                        // lösche knoten zugehörig zu arm_index_global in MS_global
                        MS_global.erase(it_global);

                        // erhöhe um 1
                        // erhöhe bei arms_global[arm_index_global] k um eins
                        arms_global[arm_index_global].update_k(1);

                        // update bei arms_global[index_global] r um den aktuellen reward (lässt
                        // sich als diff berechnen)
                        arms_global[arm_index_global].update_r(r_after_update - r_before_update);

                        // füge neu zu MS_global hinzu
                        MS_global.insert(MS_element(var_global, arms_global.at(var_global).get_r() / arms_global.at(var_global).get_k()));


                        // global ende

                        //////////////
                        if (sum_arm_k() % 100 == 0) {
                                save_solution();
                        }// save solution
                        if (sum_arm_k() == max_sim) {
                                return;
                        }
                        //////////////

                        MS_vec[i].insert(MS_element(best_individual_arm_indices[i], arms_vec[i].at(best_individual_arm_indices[i]).get_r() / arms_vec[i].at(best_individual_arm_indices[i]).get_k()));
                }
        }
}

void PSOMAB::save_solution() {
        // For UCB-normalized approach only
        int max_number_pulls = -1000000000;

        auto it = MS_global.begin();
        int return_index1 = 0;// xxx max pulls
        for (int o = 0; o < MS_global.size(); o++) {
                int arm_index = (*it).arm_index;

                // For UCB-normalized approach only
                if (arms_global.at(arm_index).get_k() > max_number_pulls) {
                        max_number_pulls = arms_global.at(arm_index).get_k();
                }
                // For UCB-normalized approach only

                it++;
        }

        // For UCB-normalized approach only
        double ucb_norm_min = 1000000000;
        double ucb_norm_max = -1000000000;

        auto it_norm = MS_global.begin();

        bool last_element_reached = false;
        while (!last_element_reached) {
                int arm_index = (*it_norm).arm_index;
                if (arms_global.at(arm_index).get_k() == max_number_pulls) {
                        last_element_reached = true;
                }
                if (arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k() < ucb_norm_min) {
                        ucb_norm_min = arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k();
                }
                if (arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k() > ucb_norm_max) {
                        ucb_norm_max = arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k();
                }

                it_norm++;
        }

        // For UCB-normalized approach only
        auto it_norm2 = MS_global.begin();
        // ucb_norm_min; //best
        // ucb_norm_max; //worst

        int return_index2 = 0;
        last_element_reached = false;
        double best_ucb_value = 1000000000;
        while (!last_element_reached) {
                int arm_index = (*it_norm2).arm_index;
                if (arms_global.at(arm_index).get_k() == max_number_pulls) {
                        last_element_reached = true;
                }
                if (1 - (ucb_norm_max - arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k()) / (ucb_norm_max - ucb_norm_min) + sqrt(2 * log(sum_arm_k()) / arms_global.at(arm_index).get_k()) < best_ucb_value) {
                        best_ucb_value = 1 - (ucb_norm_max - arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k()) / (ucb_norm_max - ucb_norm_min) + sqrt(2 * log(sum_arm_k()) / arms_global.at(arm_index).get_k());
                        return_index2 = arm_index;
                }
                if (ucb_norm_max == ucb_norm_min) {
                        return_index2 = arm_index;
                }// sonst wäre Nenner W_k - B_k null und in oberer if bedingung würde inf
                // < minvalue2 stehen
                it_norm2++;
        }

        // no noise
        // TRUE VALUE EINFACH AUF EINEN BELIEBIGEN WERT SETZEN; FALLS SIMULATION ZU
        // RECHENINTENSIV IST UND EXAKTER WERT OHNEHIN NICHT BEKANNT/BESTIMMBAR
        double true_value =
            arms_global.at(return_index2).function_value();

        // falls nach max anzahl an simulation observations abgebrochen wird
        // (wird sim_counter an erster stelle angezeigt)
        best_solutions.emplace_back(sum_arm_k(), arms_global.at(return_index2).get_action_vector(), arms_global.at(return_index2).get_k(), arms_global.at(return_index2).get_r() / arms_global.at(return_index2).get_k(), true_value);
}

Eigen::VectorXi generate_unique_solution(std::vector<Eigen::VectorXi> &solutions, Eigen::VectorXi x_lb, Eigen::VectorXi x_ub, int dimension) {
        Eigen::VectorXi v(dimension);
        // generate random solutions as long as they are not unique
        while (true) {
                for (int j = 0; j < dimension; j++) {
                        std::uniform_int_distribution<int> uniform_dist(x_lb(j), x_ub(j));
                        v(j) = uniform_dist(generator);
                }
                if (std::find(solutions.begin(), solutions.end(), v) == solutions.end()) {
                        break;
                }
        }
        return v;
}

PSOMAB::PSOMAB(std::function<double(Eigen::VectorXi)> func, unsigned long max_gen, int pop_s, unsigned seed, Eigen::VectorXi x_lb, Eigen::VectorXi x_ub, int D) : opti_func{std::move(func)}, max_sim{max_gen}, m{pop_s}, vec_x_min{x_lb}, vec_x_max{x_ub}, dimension{D} {

        //The following procedure ensures that only unique solutions are generated in the first iteration.
        for (int i = 0; i < pop_s; i++) {

                // initialize vector of solutions
                Eigen::VectorXi v(dimension);
                // generate random solutions as long as they are not unique
                v = generate_unique_solution(init_solutions, x_lb, x_ub, dimension);
                init_solutions.push_back(v);//required to check wheather all elements are unique
                // add arm to "arms", i.e. where all arms are stored
                Arm new_arm(opti_func, v, 0);// 0 = cost info, last element (0) is actually not necessary

                // initialize velocity vector
                Eigen::VectorXi init_velocity;
                init_velocity.setZero(dimension);
                velocity.push_back(init_velocity);

                // erzeuge für jeden Partikel einen (lokalen) LUT und Füge (lokalen) LUT dem Vektor aller lokalen LUTs hinzu
                LUT lookuptree_temp;
                lookuptree_vec.push_back(lookuptree_temp);

                // insert into Lookuptree (LUT)
                // Berechne "unique integer" bzw. search key/index
                int128_t search_index = calc_solution_code(new_arm.get_action_vector());
                // Füge einen neuen Knoten mit (search index, 0) dem lokalen LUT hinzu. "0" deshalb, da es der erste Knoten ist.
                lookuptree_vec.at(i).insert(0, search_index);

                // x_i
                current_particles.push_back(new_arm);

                // Vektor an Armen
                std::vector<Arm> arms_temp;
                // arms_vec[i]: Arm-Gedächtnis
                arms_vec.push_back(arms_temp);
                // (Arm-Vektor) des i-ten Partikels
                // Füge Arm "arms.at(i)" dem Gedächtnis des i-ten Partikels hinzu
                arms_vec[i].push_back(new_arm);

                // Ziehe entsprechenden arm 0: Es gibt arms.size() speicher_listen, in jeder liste wird hier nur das 1. Element befüllt.
                arms_vec[i].at(0).pull_arm();

                // current sample mean
                double Q_PSO = arms_vec[i].at(0).get_r() / arms_vec[i].at(0).get_k();

                // erzeuge für jeden Partikel einen (lokalen) SAT
                std::multiset<MS_element, std::less<>> MS_temp;
                // Füge (lokalen) SAT dem Vektor aller lokalen SATs hinzu
                MS_vec.emplace_back(MS_temp);
                // Füge einen neuen Knoten mit (Q:PSO:sample mean, 0) dem lokalen SAT hinzu. 0: Index des ersten Arms (in jeder der arms.size() Listen = diese sind partikelspezifisch)
                MS_vec[i].insert(MS_element(0, Q_PSO));

                /// global
                // berechne "unique integer" aka search index
                int128_t search_index_global = calc_solution_code(arms_vec[i].at(0).get_action_vector());
                // füge entsprechenden knoten in den GLOBALEN LUT
                lookuptree_global.insert(i, search_index);
                // füge den arm (zugehörig zum Knoten) in das globale Arm Gedächtnis
                arms_global.push_back(arms_vec[i].at(0));
                // füge einen entsprechenden Knoten in den GLOBALEN SAT ein (i: Index im globalen Arm Gedächtnis, Q_PSO: Sample Mean) hier i oben 0, da hier ->globaler <- Baum aufgebaut wird
                MS_global.insert(MS_element(i, Q_PSO));
        }
}
std::vector<solution> PSOMAB::getBest_solutions() {
        return best_solutions;
}
