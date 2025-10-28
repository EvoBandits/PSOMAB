#ifndef PSOMAB_TESTS_PROBLEMS_TWO_AGENT_SUPPLY_CHAIN_TWO_AGENT_SUPPLY_CHAIN_H_
#define PSOMAB_TESTS_PROBLEMS_TWO_AGENT_SUPPLY_CHAIN_TWO_AGENT_SUPPLY_CHAIN_H_

#include "../../src/util/RandomNumber.h"
#include "Eigen/Core"

void calc_inventory_two_agent_supply_chain(int start_inventory[][8][2]);
int calc_TC_two_agent_supply_chain(int start_inventory[][8][2]);
double get_true_objective_value_two_agent_supply_chain(const Eigen::VectorXi &action_vector);
double two_agent_supply_chain(const Eigen::VectorXi &action_vector, bool noisy);

#endif//PSOMAB_TESTS_PROBLEMS_TWO_AGENT_SUPPLY_CHAIN_TWO_AGENT_SUPPLY_CHAIN_H_
