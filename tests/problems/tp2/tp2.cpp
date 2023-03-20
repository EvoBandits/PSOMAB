#include <iostream>
#include <random>
#include <vector>
#include <cmath>

#include <iomanip>
#include <fstream>
#include <string>

unsigned seed = 1; //
std::default_random_engine e(seed);


int d_min=20;
int d_max=60;


// backorder costs
std::vector<int> b_c{24,12,6,3};
// holding costs
std::vector<int> h_c{8,4,2,1};

//transportation lead times
std::vector<int> tlt_min{1,2,4,3};
std::vector<int> tlt_max{1,4,6,5};

//information lead times
std::vector<int> ilt_min{0,0,0,0};
std::vector<int> ilt_max{0,1,1,2};

int random_number(int a, int b){
        std::uniform_int_distribution<int> random_integer(a,b);
        return random_integer(e);
};


void calc_inventory(int start_inventory[][8][2]);
int calc_TC(int start_inventory[][8][2]);
double Variance(std::vector<double> samples);









// alt *****
double double_random_number(double a, double b){
        std::uniform_real_distribution<double> random_integer(a,b);
        return random_integer(e);
};

double poisson_random_number(double a){
        std::poisson_distribution<int> random_integer(a);
        return random_integer(e);
};

double normal_distribution(double a, double b){
        std::normal_distribution<double> random_integer(a,b);
        return random_integer(e);
};

//****** alt kann gelöcht werden




int main()
{
        std::cout << std::fixed << std::setprecision(0);

        std::vector<int> min={191,  40};  // min x_1   // min x_1
        std::vector<int> max={360,  300};  // max x_1   // max x_2

        std::vector<std::vector<int>> action_vectors;
        std::vector<double> result_vector_mean;
        std::vector<double> result_vector_stdev;

        for(int i=min[0]; i<=max[0]; i++){  //klein s
                for(int j=min[1]; j<=max[1]; j++){ // GROß S
                        e.seed(1); //standardmäßig bei jedem Durchlauf auf 1
                        double final_r=0.0;
                        std::vector<double> results;
                        int sample_size=10'000;
                        for(int o=0; o<sample_size; o++)
                        {
                                std::vector<int> s{i,j};



                                int start_inventory [2][8][2]= { {{s[0],s[1]},   {0,0},    {0,0},    {0,0},    {0,0},    {0,0},  {0,0},   {0,0},},
                                                                {{0,0},   {0,0},    {0,0},    {0,0},    {0,0},    {0,0},  {0,0},   {0,0}}   };

                                // start_inventory [0][0][1],...,[0][0][2] denote the inventory levels of agent 1,2
                                /*---------------------------------------*/
                                // start_inventory [0][1][1],...,[0][1][2]
                                // start_inventory ....................... denote the shipments that are in transit
                                // start_inventory [0][7][1],...,[0][7][2]
                                /*---------------------------------------*/
                                // start_inventory [1][0][1],...,[1][0][2]
                                // start_inventory ....................... denote the orders that are in transit
                                // start_inventory [1][7][1],...,[1][7][2]


                                if(o==1){
                                        //std::cout << "Evaluated base-stock level vector: " << s[0] << " "<< s[1]  <<  std::endl;

                                        //std::cout <<  std::endl << "Calculation progress:" <<  std::endl;
                                }


                                //if(o%(sample_size/10)==0){std::cout <<o/(sample_size/100) << " %"<< std::endl;}


                                int period_number=1'200; // Supply Chain Horizon
                                double reward{0.0};
                                for(int t=0; t<period_number; t++){
                                        calc_inventory(start_inventory);
                                        reward=reward+calc_TC(start_inventory);
                                }
                                results.push_back(reward);
                        }
                        double sum = std::accumulate(results.begin(), results.end(), 0.0);
                        double mean = sum / results.size();

                        double var = Variance(results);
                        double stdev = sqrt(var);

                        std::cout <<std::fixed<<"x_1: "<<i << "  x_2: " << j <<"  mean: "<< mean<< "   stdev: "<<stdev<< std::endl;

                        result_vector_mean.push_back(mean);
                        result_vector_stdev.push_back(stdev);
                        std::vector<int> a_v = {i,j};
                        action_vectors.push_back(a_v);

                }
        }



        std::ofstream result_save;
        //std::string string_save="H:/GMAB2/.....".csv";  C:/Users/preilden/Downloads/Results/results.
        std::string string_save="C:/Users/preilden/Downloads/SS/results2.csv";
        result_save.open(string_save);
        for(int q=0; q<result_vector_mean.size(); q++){
                result_save <<std::fixed<< std::setprecision(2)<< action_vectors[q][0] <<";"<< action_vectors[q][1] <<";" <<result_vector_mean[q]<< ";" <<result_vector_stdev[q] << std::endl;
        }

        result_save.close();




        return 0;
}















void calc_inventory(int start_inventory[][8][2]){

        int information_lead_times[2]= {0,0}; // Information lead time that an order takes from agent i to agent i+1
        int transportation_lead_times[2]={0,0}; // Transportation lead time that a shipment takes from agent i+1 to agent i

        int incoming_orders[3]= {0,0,   0}; // Demand that arrives at agent i  [cf. ORD^IN in the paper version]
        int outgoing_shipments[2]= {0,0};  // Shipment that agent i is able to place in transit to agent i-1 // [cf. DEL^OUT in the paper version]

        // Shipments that arrive at each agent at the next time period
        int incoming_shipments[2]= {start_inventory[0][1][0],start_inventory[0][1][1]}; // [cf. DEL^IN in the paper version]
        // Inventory Level: On-hand inventory - backlogged order
        int inventory_level[2]={start_inventory[0][0][0],start_inventory[0][0][1]};


        int arr_size =8;
        for(int i=0; i<2; i++){

                for(int j=0; j<arr_size; j++){
                        if(j==0){
                                // Shipments arrive
                                start_inventory[0][j][i]=start_inventory[0][j][i]+incoming_shipments[i];
                        }else{
                                // all shipments in the supply chain are updated by 1 time period
                                if(j<arr_size-1){
                                        start_inventory[0][j][i]=start_inventory[0][j+1][i];
                                }

                        }
                        // all orders in the supply chain are updated by 1 time period
                        if(j<arr_size-1){
                                start_inventory[1][j][i]=start_inventory[1][j+1][i];
                        }



                        if(j==arr_size-1){
                                start_inventory[0][j][i]=0;
                                start_inventory[1][j][i]=0;
                        }
                }

        }


        for(int i=0; i<2; i++){
                if(i==0){
                        // sampling external customer demand
                        incoming_orders[i]=random_number(d_min,d_max);
                }

                if(ilt_min[i]==ilt_max[i]){                  // no sampling required
                        information_lead_times[i]=ilt_min[i];
                }
                else{
                        information_lead_times[i]=random_number(ilt_min[i],ilt_max[i]);
                }

                if(inventory_level[i]>0){
                        if(incoming_orders[i] < incoming_shipments[i]+inventory_level[i]){    // [cf. CASE 1 in the paper version]
                                outgoing_shipments[i]=incoming_orders[i];
                        }
                        else{                                                                 // [cf. CASE 2 in the paper version]
                                outgoing_shipments[i]=inventory_level[i]+incoming_shipments[i];
                        }
                }
                else{
                        if(incoming_orders[i]  < incoming_shipments[i]+inventory_level[i]){    // [cf. CASE 3 in the Paper] Note [-(inventory_level)] is positive!
                                outgoing_shipments[i]=incoming_orders[i] - inventory_level[i];
                        }
                        else{                                                                  // [cf. CASE 4 in the Paper]
                                outgoing_shipments[i]=incoming_shipments[i];
                        }
                }

                if(information_lead_times[i]==0){ // if the information lead time of agent i equals 0, i.e. the order incoming_order[i] (= DEL^IN_i=DEL^OUT_i) arrives immediately at agent i+1.
                        // incoming order(s) at agent i+1 (from agent i) = order immediately sent from agent i to agent i+1 + orders that were placed in previous time periods by agent i and that now arrive at agent i+1.
                        incoming_orders[i+1] = incoming_orders[i] + start_inventory[1][0][i];
                }
                else{
                        // incoming order(s) at agent i+1 (from agent i) =  orders that were placed in previous time periods by agent i and that now arrive at agent i+1.
                        incoming_orders[i+1] = start_inventory[1][0][i];

                        // the order placed by agent i (i.e. "DEL^OUT_i = DEL^IN_i = incoming_order[i]) with agent i+1 will arrive at agent i+1 in one of the next time periods, more precisely with a delay of information_lead_times[i].
                        // Thereby, this order is added to start_inventory[1][information_lead_times[i]][i]
                        start_inventory[1][information_lead_times[i]][i]+=incoming_orders[i];
                }
        }

        // updating the inventory level(s), i.e. subtracting the demand(s) in the current time period
        for(int i=0; i<2; i++){
                start_inventory[0][0][i]=start_inventory[0][0][i]-incoming_orders[i];
        }


        // shipments are placed in transit
        for(int i=0; i<2; i++){

                if(tlt_min[i]==tlt_max[i]){      // no sampling required
                        transportation_lead_times[i]=tlt_min[i];
                }
                else{
                        transportation_lead_times[i]=random_number(tlt_min[i],tlt_max[i]);
                }

                if(i<=2){
                        // shipment that is placed in transit by agent i+1 for agent i. It will arrive with a delay of transportation_lead_times[i]
                        start_inventory[0][transportation_lead_times[i]][i]+=outgoing_shipments[i+1];
                }
                else{
                        // shipment that is placed in transit by the external source for the most usptream agent, the supplier. It will arrive with a delay of transportation_lead_times[i]
                        start_inventory[0][transportation_lead_times[i]][i]+=incoming_orders[i+1];
                }
        }
}





int calc_TC(int start_inventory[][8][2]){
        return (((start_inventory[0][0][0]>0) ? start_inventory[0][0][0]*h_c[0]: (-1)*start_inventory[0][0][0]*b_c[0]) +
                ((start_inventory[0][0][1]>0) ? start_inventory[0][0][1]*h_c[1]: (-1)*start_inventory[0][0][1]*b_c[1]));
}


double Variance(std::vector<double> samples)
{
        int size = samples.size();

        double variance = 0;
        double t = samples[0]+0.0;
        for (int i = 1; i < size; i++)
        {
                t += samples[i];
                double diff = ((i + 1) * samples[i]) - t;
                variance += (diff * diff) / ((i + 1.0) *i);
        }

        return variance / (size - 1);
}
