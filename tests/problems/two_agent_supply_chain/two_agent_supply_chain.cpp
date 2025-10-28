#include "two_agent_supply_chain.h"

void calc_inventory_two_agent_supply_chain(int start_inventory[][8][2]) {
        const int dimension = 2;

        const int d_min = 20;
        const int d_max = 60;

        const int tlt_min[] = {1, 2, 4, 3};
        const int tlt_max[] = {1, 4, 6, 5};

        const int ilt_min[] = {0, 0, 0, 0};
        const int ilt_max[] = {0, 1, 1, 2};

        int information_lead_times[dimension] = {0, 0};
        int transportation_lead_times[dimension] = {0, 0};
        int incoming_orders[dimension + 1] = {0, 0, 0};
        int outgoing_shipments[dimension] = {0, 0};
        int incoming_shipments[dimension] = {start_inventory[0][1][0], start_inventory[0][1][1]};
        int inventory_level[dimension] = {start_inventory[0][0][0], start_inventory[0][0][1]};

        int arr_size = 8;
        for (int i = 0; i < dimension; ++i) {
                for (int j = 0; j < arr_size; ++j) {
                        if (j == 0) {
                                start_inventory[0][j][i] = start_inventory[0][j][i] + incoming_shipments[i];
                        } else {
                                if (j < arr_size - 1) {
                                        start_inventory[0][j][i] = start_inventory[0][j + 1][i];
                                }
                        }
                        if (j < arr_size - 1) {
                                start_inventory[1][j][i] = start_inventory[1][j + 1][i];
                        }

                        if (j == arr_size - 1) {
                                start_inventory[0][j][i] = 0;
                                start_inventory[1][j][i] = 0;
                        }
                }
        }

        for (int i = 0; i < dimension; ++i) {
                if (i == 0) {
                        incoming_orders[i] = random_uniform_int(d_min, d_max);
                }

                if (ilt_min[i] == ilt_max[i]) {
                        information_lead_times[i] = ilt_min[i];
                } else {
                        information_lead_times[i] = random_uniform_int(ilt_min[i], ilt_max[i]);
                }

                if (inventory_level[i] > 0) {
                        if (incoming_orders[i] < incoming_shipments[i] + inventory_level[i]) {
                                outgoing_shipments[i] = incoming_orders[i];
                        } else {
                                outgoing_shipments[i] = inventory_level[i] + incoming_shipments[i];
                        }
                } else {
                        if (incoming_orders[i] < incoming_shipments[i] + inventory_level[i]) {
                                outgoing_shipments[i] = incoming_orders[i] - inventory_level[i];
                        } else {
                                outgoing_shipments[i] = incoming_shipments[i];
                        }
                }

                if (information_lead_times[i] == 0) {
                        incoming_orders[i + 1] = incoming_orders[i] + start_inventory[1][0][i];
                } else {
                        incoming_orders[i + 1] = start_inventory[1][0][i];

                        start_inventory[1][information_lead_times[i]][i] += incoming_orders[i];
                }
        }

        for (int i = 0; i < dimension; ++i) {
                start_inventory[0][0][i] = start_inventory[0][0][i] - incoming_orders[i];
        }

        for (int i = 0; i < dimension; ++i) {
                if (tlt_min[i] == tlt_max[i]) {
                        transportation_lead_times[i] = tlt_min[i];
                } else {
                        transportation_lead_times[i] = random_uniform_int(tlt_min[i], tlt_max[i]);
                }

                if (i <= 2) {
                        start_inventory[0][transportation_lead_times[i]][i] += outgoing_shipments[i + 1];
                } else {
                        start_inventory[0][transportation_lead_times[i]][i] += incoming_orders[i + 1];
                }
        }
}

int calc_TC_two_agent_supply_chain(int start_inventory[][8][2]) {
        int b_c[] = {24, 12};
        int h_c[] = {8, 4};

        return (((start_inventory[0][0][0] > 0) ? start_inventory[0][0][0] * h_c[0] : (-1) * start_inventory[0][0][0] * b_c[0]) + ((start_inventory[0][0][1] > 0) ? start_inventory[0][0][1] * h_c[1] : (-1) * start_inventory[0][0][1] * b_c[1]));
}

double get_true_objective_value_two_agent_supply_chain(const Eigen::VectorXi &action_vector) {
        int start_inventory[2][8][2] = {{
                                            {action_vector[0], action_vector[1]},
                                            {0, 0},
                                            {0, 0},
                                            {0, 0},
                                            {0, 0},
                                            {0, 0},
                                            {0, 0},
                                            {0, 0},
                                        },
                                        {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}};

        int period_number = 1200;
        double reward = 0;

        for (int t = 0; t < period_number; t++) {
                calc_inventory_two_agent_supply_chain(start_inventory);
                reward = reward + calc_TC_two_agent_supply_chain(start_inventory);
        }

        return reward;
}

double two_agent_supply_chain(const Eigen::VectorXi &action_vector, bool noisy) {
        if (!noisy) {
                double sum = 0;
                for (int i = 0; i < 500; ++i) {
                        sum += get_true_objective_value_two_agent_supply_chain(action_vector);
                }
                return sum / 500;
        } else {
                return get_true_objective_value_two_agent_supply_chain(action_vector);
        }
}
