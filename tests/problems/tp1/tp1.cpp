#include <iostream>
#include <random>
#include <vector>
#include <iomanip>
#include <numeric>

unsigned seed = 1;
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


int main(){

        std::vector<double> results;
        int sample_size=10'000;
        std::cout << "Total number of supply chain simulations: " << sample_size << std::endl;
        std::cout << "Seed number used: " << seed << std::endl;
        for(int i=1; i<=sample_size;i++){


                // best base-stock level vector
                //std::vector<int> s{52,143,230,183}; //SDD1
                //std::vector<int> s{54,144,229,180}; //SDD2
                //std::vector<int> s{50,138,222,181}; //SDD3
                //std::vector<int> s{95,142,273,183}; //SDD4
                //std::vector<int> s{98,143,272,179}; //SDD5
                //std::vector<int> s{92,138,264,180}; //SDD6
                //std::vector<int> s{53,161,244,196}; //SSD1
                //std::vector<int> s{105,157,285,196};//SSD2
                std::vector<int> s{53,192}; //SSS1
                //std::vector<int> s{53,193,303,283}; //SSS2



                int start_inventory [2][8][2]= { {{s[0],s[1]},   {0,0},    {0,0},    {0,0},    {0,0},    {0,0},  {0,0},   {0,0},},
                                                {{0,0},   {0,0},    {0,0},    {0,0},    {0,0},    {0,0},  {0,0},   {0,0}}   };

                // start_inventory [0][0][1],...,[0][0][4] denote the inventory levels of agent 1,...,4
                /*---------------------------------------*/
                // start_inventory [0][1][1],...,[0][1][4]
                // start_inventory ....................... denote the shipments that are in transit
                // start_inventory [0][7][1],...,[0][7][4]
                /*---------------------------------------*/
                // start_inventory [1][0][1],...,[1][0][4]
                // start_inventory ....................... denote the orders that are in transit
                // start_inventory [1][7][1],...,[1][7][4]


                if(i==1){
                        std::cout << "Evaluated base-stock level vector: " << s[0] << " "<< s[1]  <<  std::endl;
                        /*std::cout << "Min demand: " << d_min << "   ||   Max demand: " << d_max<<  std::endl;
        std::cout << "Min transportation lead time: " << tlt_min[0] << ", " << tlt_min[1] << ", " << tlt_min[2] << ", " << tlt_min[3] <<  " (for each agent)"<<std::endl;
        std::cout << "Max transportation lead time: " << tlt_max[0] << ", " << tlt_max[1] << ", " << tlt_max[2] << ", " << tlt_max[3] <<  " (for each agent)"<<std::endl;
        std::cout << "Min information lead time: " << ilt_min[0] << ", " << ilt_min[1] << ", " << ilt_min[2] << ", " << ilt_min[3] <<  " (for each agent)"<<std::endl;
        std::cout << "Max information lead time: " << ilt_max[0] << ", " << ilt_max[1] << ", " << ilt_max[2] << ", " << ilt_max[3] <<  " (for each agent)"<<std::endl;*/

                        std::cout <<  std::endl << "Calculation progress:" <<  std::endl;
                }


                if(i%(sample_size/10)==0){std::cout <<i/(sample_size/100) << " %"<< std::endl;}


                int period_number=1'200; // Supply Chain Horizon
                double reward{0.0};
                for(int i=0; i<period_number; i++){
                        calc_inventory(start_inventory);
                        reward=reward+calc_TC(start_inventory);
                }
                results.push_back(reward);
        }


        double sum = std::accumulate(results.begin(), results.end(), 0.0);
        double mean = sum / results.size();

        double var = Variance(results);
        double stdev = sqrt(var);

        std::cout << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "mean TC: " << mean << "    sample size: "<<sample_size<<""<< std::endl;
        std::cout << "std: " << stdev << "  ||  variance: " << var << "    sample size: "<<sample_size<<""<< std::endl;

        std::getchar();
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
