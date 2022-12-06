/*
 * 
 * 
 * 
 * 
 * 
 * 
 * Boost (1_77_0) is required to run PSOMAB!
 * 
 * 
 * 
 * 
 * 
 * 
 * */
#include <cstdio>
#include "PSOMAB.h"
#include <iostream>
#include <iomanip>
#include <queue> 
#include <random>
#include <chrono> 
#include <fstream>
#include <string>

#include "inventory_problem.h"




//////////////////////////////// required for plotting Figure 1 (b) //////////////
std::vector<std::vector<double>> history_matrix(100, std::vector<double> (100, 0));

void save_average_number_of_visits(std::vector<std::vector<int>> history, double MR, double CR, int m, unsigned seed){
    for(int q=0; q<history.size(); q++){
        history_matrix[(history[q][1]-1)][(history[q][2]-1)]+=history[q][0]; // history[q][0] == observation number // history[q][x] = x. action geht von 1 bis 100; (deshalb -1 für index)
    }    
    
    if(seed==400){
        std::string file_name="ResultHistory__MR_"+std::to_string(MR)+"__CR_"+std::to_string(CR)+"__m_"+std::to_string(m)+".csv";

        std::ofstream result_save; 
        result_save.open(file_name);
        for(int i=0; i<100; i++){
            for(int j=0; j<100; j++){
                result_save <<   history_matrix[i][j]/400.0 << ";"<< i+1 <<";" << j+1 << std::endl;
            }
        }

        result_save.close(); 
    }
}
/////////////////////////////////////////////




void save_runs(std::vector<solution> run_history, double MR, double CR, int m ,unsigned seed){ //saves individual runs (e.g. 1 of 400); 
//In the FORM: Total Replication Number || Action vector || Mean Value  || Replication Number of the respective solution

    
    std::string dir ="PEASE_ENTER_PATH/Runs/";
    std::string file_name= dir + "MR_"+std::to_string(MR)+"__CR_"+std::to_string(CR)+"__m_"+std::to_string(m) +"__seed_"+ std::to_string(seed) +".csv";
    
    std::ofstream result_save; 
    result_save.open(file_name);    
    
    for(int q=0; q<run_history.size(); q++){

        result_save <<  run_history[q].obs_number << ";" ;
        
        for(int i=0; i<run_history[q].x.size(); i++){
            result_save <<  run_history[q].x[i] << ";" ;
        }
        result_save <<  run_history[q].mean_func_val << ";" << run_history[q].true_func_val << ";" << run_history[q].N << std::endl;
    }
    result_save.close(); 
}


void save_diversity(std::vector<std::vector<double>> diversity, double MR, double CR, int m ,unsigned seed){ // saves the average standard deviations within the best m solutions.
// for this purpose, the StdDev is calculated for each x1, x2... etc. within the best m solutions. Result => Vector of m StdDevs. This vector is now averaged /normalized by the number of dimensions

    std::string dir ="PEASE_ENTER_PATH/Diversity/";
    std::string file_name= dir + "MR_"+std::to_string(MR)+"__CR_"+std::to_string(CR)+"__m_"+std::to_string(m) +"__seed_"+ std::to_string(seed) +".csv";
    std::ofstream result_save; 
    result_save.open(file_name);    
    
    for(int q=0; q<diversity.size(); q++){
        result_save <<  diversity[q][0] << ";" << diversity[q][1]<< std::endl;
    }
    result_save.close(); 
}


void save_timestamps(std::vector<std::vector<long long int>> timestamps, double MR, double CR, int m ,unsigned seed){ // saves the timestamps for fixed sizes of V_k.
// Timestamps are available in the form: Time in MS since 1970, size V_k, number of iterations at that time.

    std::string dir ="PEASE_ENTER_PATH/Runtime/";
    std::string file_name= dir + "MR_"+std::to_string(MR)+"__CR_"+std::to_string(CR)+"__m_"+std::to_string(m) +"__seed_"+ std::to_string(seed) +".csv";
    std::ofstream result_save; 
    result_save.open(file_name);    
    
    for(int q=1; q<timestamps.size(); q++){
        int delta_ms=timestamps[q][0]-timestamps[q-1][0];
        int deta_iterations=timestamps[q][2]-timestamps[q-1][2];
        int V_k=timestamps[q][1];
        double avg_time_per_iteration=1.0*delta_ms/deta_iterations;
        result_save <<  V_k << ";" << avg_time_per_iteration<< std::endl;
    }
    result_save.close(); 
}



        
        

int main(){
    
    using namespace std::chrono; 
    
    auto start = high_resolution_clock::now();     
    

    std::vector<std::vector<double>> mean_result;
    std::vector<std::vector<double>> true_result;
    std::vector<double> final_mean_values;
    std::vector<double> final_true_values;    
    
    std::vector<int> simulation_observations;
    
    //TP1
    
    Eigen::VectorXi x_lb(2);
    x_lb << 1, 1;
    Eigen::VectorXi x_ub(2);
    x_ub << 100, 100;
    
    
    //TP_2
    /*
    std::vector<int> x_lb{1,1,1,1,1,1,1,1};
    std::vector<int> x_ub{20,20,20,20,20,20,20,20};*/
    
    //TP3
    /*
    std::vector<int> x_lb{0,0};
    std::vector<int> x_ub{10000,10000};*/
    
    
    //TP4_D05 - TP4_20
    /*
    std::vector<int> x_lb{-100,-100,-100,-100,-100};
    std::vector<int> x_ub{100,100,100,100,100};*/
    /*
    std::vector<int> x_lb{-100,-100,-100,-100,-100, -100,-100,-100,-100,-100};
    std::vector<int> x_ub{100,100,100,100,100, 100, 100, 100, 100, 100};*/
    /*
    std::vector<int> x_lb{-100,-100,-100,-100,-100, -100,-100,-100,-100,-100,-100,-100,-100,-100,-100};
    std::vector<int> x_ub{100,100,100,100,100, 100, 100, 100, 100, 100, 100,100,100,100,100};*/
    /*
    std::vector<int> x_lb{-100,-100,-100,-100,-100, -100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100};
    std::vector<int> x_ub{100,100,100,100,100, 100, 100, 100, 100, 100, 100,100,100,100,100,100,100,100,100,100};*/
    


   
    int number_of_samples=400;
    
    int m=10;
    double MR=0.25;
    double CR=1.0;
    
    for(unsigned i=1; i<=number_of_samples; i++){
        unsigned seed=i;
        
        //ANMERKUNG: MR, CR sowie sigma_d (mutation width) bei PSOMAB ohne Funktion (dies sind ursprüngliche GMAB Parameter)
        
        // run the min-problems TP4_D05 - TP4_20           ... and TP1 as well as TP3   (Please change the objective function within Arm.cpp if you change your test problem)
        PSOMAB instance = PSOMAB(funccc_value, 10000, m, 0, 1, seed, x_lb, x_ub); //itarations, population_size, mutation_rate, crossover_rate, mutation_span , objective ( 0=min, 1=max), 1== sim obs
        //(budget, m, mutatation_probability, crossover_probability, sigma_d (mutation width), objective (0:min, 1:max), modus (0: budget=number of GMAB iterations, 1: budget=number of simulation observations), seed, x_lb, x_ub)
        
        
        // run the max-problem TP2  (Please change the objective function within Arm.cpp if you change your test problem)
        //PSOMAB instance(100000, m, MR, CR, 0.1, 1, 1, seed, x_lb, x_ub);    
        
        
        std::cout << "run "<<  i << "/"<<number_of_samples<<" TP1" << std::endl;
        instance.run();  

        std::vector<double>mean_values;
        std::vector<double>true_values;
        
        
        
        


        


        for(int k=0; k<instance.best_solutions.size();k++){
            mean_values.push_back(instance.best_solutions.at(k).mean_func_val);
            true_values.push_back(instance.best_solutions.at(k).true_func_val);
            std::cout <<i << "/"<<number_of_samples << " runs|"<<"budget:"<<instance.best_solutions.at(k).obs_number<<" | true:"<< instance.best_solutions.at(k).true_func_val << "  mean:"<<instance.best_solutions.at(k).mean_func_val<<"  ";
            std::cout <<"N: " <<instance.best_solutions.at(k).N<< "   ";
            for(int q=0; q<instance.best_solutions.at(k).x.size(); q++){
                std::cout << instance.best_solutions.at(k).x(q) << " " ;
            }
            std::cout << std::endl;
            
            if(i==number_of_samples){simulation_observations.push_back(instance.best_solutions.at(k).obs_number);} //wird nur bei einem Seed-Run benötigt um die obs_numbern zu erhalten
        }
        
        
        
        /**/ /* Save_Runs */ 
        //save_runs(instance.best_solutions, MR, CR, m, seed);
        /**/ /* Save_Runs */ /**/
        
        
        /**/  /*Save History */ 
        /*
        std::vector<std::vector<int>> history = instance.get_history();
        for(int k=0; k< history.size(); k++){
            std::cout << history[k][0] << "   |";
            for(int q=1; q<history[k].size(); q++){
                std::cout << history[k][q] << " ";
            }
            std::cout << std::endl;
        }
        save_average_number_of_visits(history, MR, CR, m, seed);*/
        /**/  /*History */ /**/         
         
        /**/ /* Save_Diversity */ 
        //save_diversity(instance.diversity, MR, CR, m, seed);
        /**/ /* Save_Runs */ /**/
        
        /**/ /* Save_Timestamps */ /**/  
        //save_timestamps(instance.timestamps, MR, CR, m, seed);
        /**/ /* Save_Runs */ /**/ 
        


        mean_result.push_back(mean_values);
        true_result.push_back(true_values);
        

    }    

    for(int j=0; j<mean_result.at(0).size(); j++){ // j=0,...,9999
        double mean_x=0.0;
        double true_x=0.0;
        for(int i=0; i<mean_result.size(); i++){
            mean_x=mean_x+mean_result.at(i).at(j);
            true_x=true_x+true_result.at(i).at(j);
        }
        mean_x=mean_x/mean_result.size();
        true_x=true_x/true_result.size();
        
        final_mean_values.push_back(mean_x);
        final_true_values.push_back(true_x);
    } 
    

    //std::string dir ="M:/....";

    std::string file_name="Result__MR_"+std::to_string(MR)+"__CR_"+std::to_string(CR)+"__m_"+std::to_string(m)+".csv";

    std::ofstream result_save; 
    result_save.open(file_name);
    for(int q=0; q<final_true_values.size(); q++){
        result_save <<   simulation_observations[q] << ";"<< final_mean_values[q] <<";" << final_true_values[q] << std::endl;
    }

    result_save.close(); 
    

    auto stop = high_resolution_clock::now(); 
    auto duration = duration_cast<microseconds>(stop - start); 
    std::cout <<"PSOMAB required "<< duration.count()/1000 << " ms runtime."<< std::endl;  
    std::getchar();
    
    return 0;
}










