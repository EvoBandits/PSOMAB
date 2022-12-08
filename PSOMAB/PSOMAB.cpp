#include "PSOMAB.h"
#include "Arm.h"
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
#include "RandomNumber.h"

using namespace boost::multiprecision;
using namespace std::chrono;

// std::uniform_int_distribution<int> uniform_int_distribution(a, b);
// std::uniform_real_distribution<double> uniform_real_distribution(a, b);

// calculates the solution code, i.e. unique integer
int128_t PSOMAB::calc_solution_code(std::vector<int> x) {
        int128_t search_index = 0;
        int dim = x.size();
        for (int i = 0; i < dim; i++) {
                int exp = ceil(log10((vec_x_max[i] - vec_x_min[i]) + 1));
                search_index = search_index + (int128_t) (pow((pow(10, exp)), (dim - 1) - i) * (x[i] - vec_x_min[i]));
        }
        return search_index;
}

//
void PSOMAB::PSO(int best_global_particle_index, int best_global_index, std::vector<Arm> best_individual_arms) {
        for (int k = 0; k < m; k++) {

                double c1 = 2.5;
                double c2 = 1;
                double v = 0.2;

                for (int g = 0; g < current_particles.at(k).get_action_vector().size(); g++) {
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

void PSOMAB::MAB(int save_solution_every_x, std::vector<int> best_individual_arm_indices, int z) {
        for (unsigned int i = 0; i < current_particles.size(); i++) {
                int128_t search_index = calc_solution_code(current_particles[i].get_action_vector());// berechne "unique integer" aka search index
                const int arm_index = lookuptree_vec[i].search(search_index);                        // Suche im lokalen LUT des i-ten partikel nach arm_index
                if (arm_index >= 0) {                                                                // alle arm_index >= 0 existieren, -1 falls arm noch nicht vorhanden existiert bereits
                        bool check_if_arm_was_in_current_arms = false;
                        if (arm_index == best_individual_arm_indices[i]) {
                                check_if_arm_was_in_current_arms = true;
                        }
                        if (!check_if_arm_was_in_current_arms) {

                                auto it = MS_vec[i].find(MS_element(arm_index, arms_vec[i].at(arm_index).get_r() / arms_vec[i].at(arm_index).get_k()));
                                // Suche im lokalen SAT des i-ten Partikel nach entsprechendem Knoten
                                int var = (*it).arm_index;
                                while (var != arm_index) {// If two nodes have the same mean value, the case var!=arm_index max occur In this case, the tree must be iterated further until var==arm_index in order to actually find the correct (SAT) node
                                        it++;
                                        var = (*it).arm_index;
                                }
                                MS_vec[i].erase(it);//  // Delete Node from Local SAT

                                double r_before_update = arms_vec[i].at(var).get_r();// // Store current mean in cache (before update)

                                // pull_arm
                                arms_vec[i].at(var).pull_arm();// pull arm
                                sim_counter += 1;              // increase total sim_counter

                                // global
                                double r_after_update = arms_vec[i].at(var).get_r();                // // Store current mean in cache (after update)
                                const int arm_index_global = lookuptree_global.search(search_index);// // Suche entsprechenden Arm im global LUT arm_index_global=suche von arms_vec[i].at(var) den index im lookuptree_global
                                auto it_global = MS_global.find(MS_element(arm_index_global,arms_global.at(arm_index_global).get_r() / arms_global.at(arm_index_global).get_k()));
                                // Suche im GLOBALEN SAT nach entsprechendem Knoten
                                int var_global = (*it_global).arm_index;
                                while (var_global != arm_index_global) {// If two nodes have the same mean value,
                                                                        // the case var!=arm_index max occur
                                        // In this case, the tree must be iterated further until
                                        // var==arm_index in order to actually find the correct (SAT) node
                                        it_global++;
                                        var_global = (*it_global).arm_index;
                                }
                                MS_global.erase(it_global);
                                // lösche knoten zugehörig zu arm_index_global in MS_global

                                arms_global[arm_index_global].update_k(1);// erhöhe um 1 // IM Globalen Arm Gedächtnis muss der sim
                                       // counter um 1 erhöht werden, da der arm (wenige zeilen
                                       // weiter oben) gezogen worden ist.
                                // erhöhe bei arms_global[arm_index_global] k um eins

                                arms_global[arm_index_global].update_r(r_after_update - r_before_update);// Im Globalen Arm Gedächtnis muss der Sample
                                                                      // mean geupdatet werden, da (wenige zeilen
                                                                      // weiter oben) der Arm gezogen worden ist.
                                // update bei arms_global[index_global] r um den aktuellen reward
                                // (lässt sich als diff berechnen)

                                MS_global.insert(MS_element(var_global,arms_global.at(var_global).get_r() / arms_global.at(var_global).get_k()));// füge neu zu MS_global hinzu
                                //////end global////////

                                if (stopping_criterion == 1) {// stop after max observations
                                        if (sim_counter % save_solution_every_x == 0) {
                                                save_solution(z);
                                        }// save solution
                                        if (sim_counter == max_iter_or_sim_number) {
                                                return;
                                        }// max_generations ist im fall von stopping_criterion==1 ein dummy
                                         // für die max anzahl an observations
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
                        Arm new_arm(opti_func, current_particles[i].get_action_vector(),0);// 0 = cost info, eigentlich nicht notwendig

                        arms_vec[i].push_back(new_arm);// füge Arm dem lokalen Arm Gedächtnis
                                                       // des i-ten Partikel zu

                        double r_before_update = arms_vec[i].back().get_r();// for global
                        // neuen Arm ziehen
                        arms_vec[i].back().pull_arm();
                        sim_counter += 1;

                        // global
                        double r_after_update = arms_vec[i].back().get_r();
                        int128_t search_index_global = calc_solution_code( arms_vec[i].back().get_action_vector());//  berechne "unique integer" aka search
                                                      //  index
                        const int arm_index_global = lookuptree_global.search( search_index_global);   // arm_index_global=suche von
                                                    // arms_vec[i].at(var) den index im
                                                    // lookuptree_GLOBAL
                        if (arm_index_global >= 0) {// falls knoten bereits im GLOBAL LUT existiert
                                auto it_global = MS_global.find(MS_element(arm_index_global,arms_global.at(arm_index_global).get_r() / arms_global.at(arm_index_global).get_k()));
                                int var_global = (*it_global).arm_index;
                                while (var_global != arm_index_global) {// Falls zwei Lösungen den
                                                                        // selben Mean Value haben,
                                        // kann prinzipiell var!=arm_index auftreten. Dann muss im Baum
                                        // weiter iteriert werden bis var==arm_index um wirklich die
                                        // richtige Lösung zu ziehen.
                                        it_global++;
                                        var_global = (*it_global).arm_index;
                                }
                                MS_global.erase(it_global);
                                // lösche knoten zugehörig zu arm_index_global in MS_global (er wird
                                // gezogen und verändert sich. für das update muss man ihn deshalb
                                // löschen)

                                arms_global[arm_index_global].update_k(1);// erhöhe um 1
                                // erhöhe bei arms_global[arm_index_global] k um eins

                                arms_global[arm_index_global].update_r(r_after_update - r_before_update);
                                // update bei arms_global[index_global] r um den aktuellen reward
                                // (lässt sich als diff berechnen)

                                MS_global.insert(MS_element(var_global, arms_global.at(var_global).get_r() / arms_global.at(var_global).get_k()));
                                // füge neu zu MS_global hinzu

                        } else {// existiert im GLOBAL LUT noch nicht

                                arms_global.push_back(arms_vec[i].back());
                                // füge den arm zu global arms hinzu

                                int new_index_global = (int) arms_global.size() - 1;
                                lookuptree_global.insert(new_index_global, search_index);
                                // füge neuen knoten in lookuptree ein

                                double Q_global = arms_global.at(new_index_global).get_r() / arms_global.at(new_index_global).get_k();
                                MS_global.insert(MS_element(new_index_global, arms_global.at(new_index_global).get_r() / arms_global.at(new_index_global).get_k()));
                                // füge neuen knoten in MS_GLOBAL ein
                        }
                        // global ende

                        //////////////
                        if (stopping_criterion == 1) {// stop after max observations
                                if (sim_counter % save_solution_every_x == 0) {
                                        save_solution(z);
                                }// save solution
                                if (sim_counter == max_iter_or_sim_number) {
                                        return;
                                }// max_generations ist im fall von stopping_criterion==1 ein dummy
                                 // für die max anzahl an observations
                        }
                        //////////////

                        // In lokalen LUT einfügen
                        int new_index = (int) arms_vec[i].size() - 1;
                        // root = (*root).InsertIntoTree(root, search_index, new_index); // fill
                        // search tree ,  (arms.size()-1) ist index des letzten Elements
                        lookuptree_vec[i].insert(new_index, search_index);

                        // In lokalen SAT einfügen
                        double Q = arms_vec[i].at(new_index).get_r() / arms_vec[i].at(new_index).get_k();
                        MS_vec[i].insert( MS_element(new_index, arms_vec[i].at(new_index).get_r() / arms_vec[i].at(new_index).get_k()));
                }
        }
}

int PSOMAB::run() {
        int save_solution_every_x = 100;// if stopping criterion = max number of sim. obs.: each 100th
            // solution is stored for "log-file" data

        for (int z = 1; z <= max_iter_or_sim_number; z++) {
                if (z == 1) {// first iteration   !!!LOGIK IN DER ERSTEN ITERATION!!!
                        for (int i = 0; i < arms.size(); i++) {

                                LUT lookuptree_temp;// erzeuge für jeden Partikel einen (lokalen) LUT
                                lookuptree_vec.push_back(lookuptree_temp);// Füge (lokalen) LUT dem Vektor aller lokalen
                                                     // LUTs hinzu
                                // insert into Lookuptree (LUT)
                                int128_t search_index = calc_solution_code(arms.at(i).get_action_vector());// Berechne "unique integer" bzw.
                                                                    // search key/index
                                lookuptree_vec.at(i).insert(0, search_index);// Füge einen neuen Knoten mit (search index, 0) dem
                                                  // lokalen LUT hinzu.  "0" deshalb, da es der erste
                                                  // Knoten ist.

                                std::vector<int> init_velocity;
                                for (int q = 0; q < arms.at(i).get_action_vector().size(); q++) {
                                        init_velocity.push_back(0);// initialise velocity with 0
                                }
                                velocity.push_back(init_velocity);

                                current_particles.push_back(arms.at(i));// x_i

                                std::vector<Arm> arms_temp;   // Vektor an Armen
                                arms_vec.push_back(arms_temp);// arms_vec[i]: Arm-Gedächtnis
                                                              // (Arm-Vektor) des i-ten Partikels
                                arms_vec[i].push_back(arms.at(i));// Füge Arm "arms.at(i)" dem Gedächtnis des i-ten
                                                // Partikels hinzu

                                arms_vec[i].at(0).pull_arm();// Ziehe entsprechenden arm 0: Es gibt arms.size()
                                                             // speicher_listen, in jeder liste wird hier nur das 1.
                                                             // Element befüllt.
                                sim_counter += 1;            // erhöhe den totalen sim obs. counter
                                //////////////
                                if (stopping_criterion == 1) {// stop after max observations
                                        if (sim_counter % save_solution_every_x == 0) {
                                                save_solution(z);
                                        }// save solution
                                        if (sim_counter == max_iter_or_sim_number) {
                                                return 0;
                                        }
                                }

                                double Q_PSO = arms_vec[i].at(0).get_r() / arms_vec[i].at(0).get_k();// current sample mean

                                std::multiset<MS_element, std::less<>> MS_temp;// erzeuge für jeden Partikel einen (lokalen) SAT
                                MS_vec.emplace_back(MS_temp);// Füge (lokalen) SAT dem Vektor aller lokalen SATs hinzu
                                MS_vec[i].insert(MS_element(0,Q_PSO));// // Füge einen neuen Knoten mit (Q:PSO:sample
                                                       // mean, 0) dem lokalen SAT hinzu.     0 : Index
                                                       // des ersten Arms (in jeder der arms.size()
                                                       // Listen = diese sind partikelspezifisch)

                                /// global
                                int128_t search_index_global = calc_solution_code(arms_vec[i].at(0).get_action_vector());// berechne "unique integer"
                                                                           // aka search index
                                lookuptree_global.insert(i, search_index);// füge entsprechenden knoten in den GLOBALEN LUT
                                arms_global.push_back(arms_vec[i].at(0));// füge den arm (zugehörig zum Knoten) in das
                                                       // GLOBALE Arm gedächntis
                                MS_global.insert(MS_element(i,Q_PSO));// füge einen entsprechenden Knoten in den GLOBALEN SAT ein
                                            // (i: Index im globalen Arm Gedächtnis, Q_PSO: Sample
                                            // Mean)   //  hier i oben 0, da hier ->globaler <- Baum
                                            // aufgebaut wird
                        }
                }

                // After first iteration
                if (z == 1) {
                        save_solution(
                            z);// save the solution that is currently considered to be the best
                }

                std::vector<Arm> best_individual_arms;       //
                std::vector<int> best_individual_arm_indices;//

                double best_global_Q;
                best_global_Q = 1000000000;// hier muss NICHT zwischen min oder max
                                           // optimierungsproblem unterschieden werden.
                //                         in Arm Funktion werden max probleme mit (-1)
                //                         multiplikation zu äquiv. min problemen gemacht,
                //                         deshalb ist hier ein hoher positiver Q Wert immer
                //                         richtig.
                int best_global_index;         // index des glabal best arm
                int best_global_particle_index;// Welches der insgesamt m Partel den global
                                               // best arm enthält

                for (int k = 0; k < m; k++) {

                        auto it_PSO = MS_vec[k].begin();

                        int arm_index = (*it_PSO).arm_index;

                        best_individual_arms.push_back(arms_vec[k].at(arm_index));// weise es den Armen der aktuellen Iteration zu
                        best_individual_arm_indices.push_back(arm_index);

                        if ((*it_PSO).Q < best_global_Q) {
                                best_global_Q = (*it_PSO).Q;
                                best_global_particle_index = k;
                                best_global_index = (*it_PSO).arm_index;
                        }
                }

                PSOMAB::PSO(best_global_particle_index, best_global_index, best_individual_arms);

                MAB(save_solution_every_x, best_individual_arm_indices, z);

                // Die m besten Arme werden in jeder Iteration erneut gezogen um bessere
                // Sample Means zu erhalten. Das passiert in der folgenden For loop
                for (unsigned int i = 0; i < current_particles.size(); i++) {
                        auto it = MS_vec[i].find(MS_element(best_individual_arm_indices[i],arms_vec[i].at(best_individual_arm_indices[i]).get_r() / arms_vec[i].at(best_individual_arm_indices[i]).get_k()));
                        int var = (*it).arm_index;
                        while (var != best_individual_arm_indices[i]) {// Falls zwei Lösungen den selben
                                                                       // Mean Value haben,
                                // kann prinzipiell var!=arm_index auftreten. Dann muss im Baum weiter
                                // iteriert werden bis var==arm_index um wirklich die richtige Lösung zu
                                // ziehen.
                                it++;
                                var = (*it).arm_index;
                        }
                        MS_vec[i].erase(it);

                        double r_before_update = arms_vec[i].at(best_individual_arm_indices[i]).get_r();// für global

                        arms_vec[i].at(best_individual_arm_indices[i]).pull_arm();
                        sim_counter += 1;

                        // global

                        double r_after_update = arms_vec[i].at(best_individual_arm_indices[i]).get_r();
                        int128_t search_index_global = calc_solution_code(arms_vec[i].at(best_individual_arm_indices[i]).get_action_vector());
                        const int arm_index_global = lookuptree_global.search(search_index_global);

                        auto it_global = MS_global.find(MS_element(arm_index_global, arms_global.at(arm_index_global).get_r() / arms_global.at(arm_index_global).get_k()));
                        int var_global = (*it_global).arm_index;
                        while (var_global != arm_index_global) {// Falls zwei Lösungen den selben
                                                                // Mean Value haben,
                                // kann prinzipiell var!=arm_index auftreten. Dann muss im Baum weiter
                                // iteriert werden bis var==arm_index um wirklich die richtige Lösung zu
                                // ziehen.
                                it_global++;
                                var_global = (*it_global).arm_index;
                                // std::cout << "stuck " << var_global<< std::endl;
                        }
                        MS_global.erase(it_global);
                        // lösche knoten zugehörig zu arm_index_global in MS_global

                        arms_global[arm_index_global].update_k(1);// erhöhe um 1
                        // erhöhe bei arms_global[arm_index_global] k um eins

                        arms_global[arm_index_global].update_r(r_after_update - r_before_update);
                        // update bei arms_global[index_global] r um den aktuellen reward (lässt
                        // sich als diff berechnen)

                        MS_global.insert(MS_element(var_global, arms_global.at(var_global).get_r() / arms_global.at(var_global).get_k()));
                        // füge neu zu MS_global hinzu
                        //

                        // global ende

                        //////////////
                        if (stopping_criterion == 1) {// stop after max observations
                                if (sim_counter % save_solution_every_x == 0) {
                                        save_solution(z);
                                }// save solution
                                if (sim_counter == max_iter_or_sim_number) {
                                        return 0;
                                }// max_generations ist im fall von stopping_criterion==1 ein dummy für
                                 // die max anzahl an observations
                        }
                        //////////////

                        MS_vec[i].insert(MS_element(best_individual_arm_indices[i],arms_vec[i].at(best_individual_arm_indices[i]).get_r() / arms_vec[i].at(best_individual_arm_indices[i]).get_k()));
                }

                // at the end of each iteration
                if (z % 1 == 0) {
                        if (stopping_criterion == 0) {// if stopping criterion = max number of iterations
                                save_solution(
                                    z);// save the solution that is currently considered to be the best
                        }
                }

                ///////// only to log the timestamps, no actual PSOMAB function
                // save_timesteps(z);
                /////////
        }
        return 0;// dummy return
}

void PSOMAB::save_solution(int z) {// z iterationszahl
        // For UCB-normalized approach only
        int max_number_pulls = -1000000000;
        // For UCB-normalized approach only

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
                if (1 - (ucb_norm_max - arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k()) / (ucb_norm_max - ucb_norm_min) + sqrt(2 * log(sim_counter) / arms_global.at(arm_index).get_k()) < best_ucb_value) {
                        best_ucb_value = 1 - (ucb_norm_max - arms_global.at(arm_index).get_r() / arms_global.at(arm_index).get_k()) / (ucb_norm_max - ucb_norm_min) + sqrt(2 * log(sim_counter) / arms_global.at(arm_index).get_k());
                        return_index2 = arm_index;
                }
                if (ucb_norm_max == ucb_norm_min) {
                        return_index2 = arm_index;
                }// sonst wäre Nenner W_k - B_k null und in oberer if bedingung würde inf
                // < minvalue2 stehen
                it_norm2++;
        }

        double true_value =
            arms_global.at(return_index2).function_value();// no noise
        /// TRUE VALUE EINFACH AUF EINEN BELIEBIGEN WERT SETZEN; FALLS SIMULATION ZU
        /// RECHENINTENSIV IST UND EXAKTER WERT OHNEHIN NICHT BEKANNT/BESTIMMBAR

        if (stopping_criterion == 1) {// falls nach max anzahl an simulation observations abgebrochen wird
                                      // (wird sim_counter an erster stelle angezeigt)
                best_solutions.emplace_back(sim_counter, arms_global.at(return_index2).get_action_vector(),arms_global.at(return_index2).get_k(),arms_global.at(return_index2).get_r() / arms_global.at(return_index2).get_k(),true_value);
        } else {// d.h. falls nach max anzahl an iterationen abgebrochen wird (wird z
                // = iterationszahl an erster stelle angezeigt)
                best_solutions.emplace_back(z, arms_global.at(return_index2).get_action_vector(),arms_global.at(return_index2).get_k(),arms_global.at(return_index2).get_r() / arms_global.at(return_index2).get_k(),true_value);
        }
}

PSOMAB::PSOMAB(std::function<double(std::vector<int>)> func, unsigned long max_gen, int pop_s, int stopping_criterion, unsigned seed, std::vector<int> x_lb, std::vector<int> x_ub, int D) : opti_func{std::move(func)}, max_iter_or_sim_number{max_gen}, m{pop_s}, stopping_criterion(stopping_criterion), vec_x_min{x_lb}, vec_x_max{x_ub}, dimension{D} {

        // The following procedure ensures that only unique solutions are generated in
        // the first iteration.
        for (int i = 0; i < pop_s; i++) {
                std::vector<int> v;
                for (int j = 0; j < dimension; j++) {
                        std::uniform_int_distribution<int> uniform_int_distribution(x_lb[j], x_ub[j]);
                        v.push_back(uniform_int_distribution(generator));
                }

                // Check if v is unique, and if not generate new vector
                while (init_solutions.count(v) > 0) {
                        v.clear();
                        for (int j = 0; j < dimension; j++) {
                                std::uniform_int_distribution<int> uniform_int_distribution(x_lb[j], x_ub[j]);
                                v.push_back(uniform_int_distribution(generator));
                        }
                }
                init_solutions.insert(v);// required to check wheather all elements are unique

                // add arm to "arms", i.e. where all arms are stored
                Arm new_arm(opti_func, v,0);// 0 = cost info, last element (0) is actually not necessary
                arms.push_back(new_arm);
        }
}
