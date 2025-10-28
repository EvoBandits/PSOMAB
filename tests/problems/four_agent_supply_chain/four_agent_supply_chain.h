#ifndef PSOMAB_TESTS_PROBLEMS_FOUR_AGENT_SUPPLY_CHAIN_FOUR_AGENT_SUPPLY_CHAIN_H_
#define PSOMAB_TESTS_PROBLEMS_FOUR_AGENT_SUPPLY_CHAIN_FOUR_AGENT_SUPPLY_CHAIN_H_

#include "../../src/util/RandomNumber.h"
#include "Eigen/Core"

void calc_inventory_four_agent_supply_chain(int start_inventory[][8][4]);
int calc_TC_four_agent_supply_chain(int start_inventory[][8][4]);
double get_true_objective_value_four_agent_supply_chain(const Eigen::VectorXi &action_vector);
double four_agent_supply_chain(const Eigen::VectorXi &action_vector, bool noisy);

#endif//PSOMAB_TESTS_PROBLEMS_FOUR_AGENT_SUPPLY_CHAIN_FOUR_AGENT_SUPPLY_CHAIN_H_
