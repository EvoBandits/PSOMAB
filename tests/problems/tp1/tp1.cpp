#include "tp1.h"

Eigen::Vector2i tp1_lb(20, 40);
Eigen::Vector2i tp1_ub(360, 300);

void calc_inventory_tp1(int start_inventory[][8][2]) {
        const int d_min = 20;
        const int d_max = 60;

        //transportation lead times
        const int tlt_min[] = {1, 2, 4, 3};
        const int tlt_max[] = {1, 4, 6, 5};

        //information lead times
        const int ilt_min[] = {0, 0, 0, 0};
        const int ilt_max[] = {0, 1, 1, 2};

        // Information lead time that an order takes from agent i to agent i+1
        int information_lead_times[tp1_dim] = {0, 0};

        // Transportation lead time that a shipment takes from agent i+1 to agent i
        int transportation_lead_times[tp1_dim] = {0, 0};

        // Demand that arrives at agent i  [cf. ORD^IN in the paper version]
        int incoming_orders[tp1_dim] = {0, 0};

        // Shipment that agent i is able to place in transit to agent i-1 // [cf. DEL^OUT in the paper version]
        int outgoing_shipments[tp1_dim] = {0, 0};

        // Shipments that arrive at each agent at the next time period [cf. DEL^IN in the paper version]
        int incoming_shipments[tp1_dim] = {start_inventory[0][1][0], start_inventory[0][1][1]};
        // Inventory Level: On-hand inventory - backlogged order
        int inventory_level[tp1_dim] = {start_inventory[0][0][0], start_inventory[0][0][1]};

        int arr_size = 8;
        for (int i = 0; i < tp1_dim; ++i) {
                for (int j = 0; j < arr_size; ++j) {
                        if (j == 0) {
                                // Shipments arrive
                                start_inventory[0][j][i] = start_inventory[0][j][i] + incoming_shipments[i];
                        } else {
                                // all shipments in the supply chain are updated by 1 time period
                                if (j < arr_size - 1) {
                                        start_inventory[0][j][i] = start_inventory[0][j + 1][i];
                                }
                        }
                        // all orders in the supply chain are updated by 1 time period
                        if (j < arr_size - 1) {
                                start_inventory[1][j][i] = start_inventory[1][j + 1][i];
                        }

                        if (j == arr_size - 1) {
                                start_inventory[0][j][i] = 0;
                                start_inventory[1][j][i] = 0;
                        }
                }
        }

        for (int i = 0; i < tp1_dim; ++i) {
                if (i == 0) {
                        // sampling external customer demand
                        incoming_orders[i] = random_uniform_int(d_min, d_max);
                }

                if (ilt_min[i] == ilt_max[i]) {
                        // no sampling required
                        information_lead_times[i] = ilt_min[i];
                } else {
                        information_lead_times[i] = random_uniform_int(ilt_min[i], ilt_max[i]);
                }

                if (inventory_level[i] > 0) {
                        if (incoming_orders[i] < incoming_shipments[i] + inventory_level[i]) {
                                // [cf. CASE 1 in the paper version]
                                outgoing_shipments[i] = incoming_orders[i];
                        } else {
                                // [cf. CASE 2 in the paper version]
                                outgoing_shipments[i] = inventory_level[i] + incoming_shipments[i];
                        }
                } else {
                        if (incoming_orders[i] < incoming_shipments[i] + inventory_level[i]) {
                                // [cf. CASE 3 in the Paper] Note [-(inventory_level)] is positive!
                                outgoing_shipments[i] = incoming_orders[i] - inventory_level[i];
                        } else {
                                // [cf. CASE 4 in the Paper]
                                outgoing_shipments[i] = incoming_shipments[i];
                        }
                }

                if (information_lead_times[i] == 0) {
                        // if the information lead time of agent i equals 0, i.e. the order incoming_order[i] (= DEL^IN_i=DEL^OUT_i) arrives immediately at agent i+1.
                        // incoming order(s) at agent i+1 (from agent i) = order immediately sent from agent i to agent i+1 + orders that were placed in previous time periods by agent i and that now arrive at agent i+1.
                        incoming_orders[i + 1] = incoming_orders[i] + start_inventory[1][0][i];
                } else {
                        // incoming order(s) at agent i+1 (from agent i) =  orders that were placed in previous time periods by agent i and that now arrive at agent i+1.
                        incoming_orders[i + 1] = start_inventory[1][0][i];

                        // the order placed by agent i (i.e. "DEL^OUT_i = DEL^IN_i = incoming_order[i]) with agent i+1 will arrive at agent i+1 in one of the next time periods, more precisely with a delay of information_lead_times[i].
                        // Thereby, this order is added to start_inventory[1][information_lead_times[i]][i]
                        start_inventory[1][information_lead_times[i]][i] += incoming_orders[i];
                }
        }

        // updating the inventory level(s), i.e. subtracting the demand(s) in the current time period
        for (int i = 0; i < tp1_dim; ++i) {
                start_inventory[0][0][i] = start_inventory[0][0][i] - incoming_orders[i];
        }

        // shipments are placed in transit
        for (int i = 0; i < tp1_dim; ++i) {
                if (tlt_min[i] == tlt_max[i]) {
                        // no sampling required
                        transportation_lead_times[i] = tlt_min[i];
                } else {
                        transportation_lead_times[i] = random_uniform_int(tlt_min[i], tlt_max[i]);
                }

                if (i <= 2) {
                        // shipment that is placed in transit by agent i+1 for agent i. It will arrive with a delay of transportation_lead_times[i]
                        start_inventory[0][transportation_lead_times[i]][i] += outgoing_shipments[i + 1];
                } else {
                        // shipment that is placed in transit by the external source for the most usptream agent, the supplier. It will arrive with a delay of transportation_lead_times[i]
                        start_inventory[0][transportation_lead_times[i]][i] += incoming_orders[i + 1];
                }
        }
}

int calc_TC_tp1(int start_inventory[][8][2]) {
        // backorder costs
        int b_c[] = {24, 12, 6, 3};
        // holding costs
        int h_c[] = {8, 4, 2, 1};

        return (((start_inventory[0][0][0] > 0) ? start_inventory[0][0][0] * h_c[0] : (-1) * start_inventory[0][0][0] * b_c[0]) + ((start_inventory[0][0][1] > 0) ? start_inventory[0][0][1] * h_c[1] : (-1) * start_inventory[0][0][1] * b_c[1]));
}

double get_true_objective_value_tp1(const Eigen::VectorXi &action_vector) {
        int start_inventory[2][8][tp1_dim] = {{
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

        // Supply Chain Horizon
        int period_number = 1200;
        double reward = 0;

        for (int i = 0; i < period_number; ++i) {
                calc_inventory_tp1(start_inventory);
                reward = reward + calc_TC_tp1(start_inventory);
        }

        return reward;
}

double tp1(const Eigen::VectorXi &action_vector, int noise_level) {
        if (noise_level == 0) {
                double sum = 0;
                for (int i = 0; i < 500; ++i) {
                        sum += get_true_objective_value_tp1(action_vector);
                }
                return sum / 500;
        } else {
                return get_true_objective_value_tp1(action_vector);
        }
}