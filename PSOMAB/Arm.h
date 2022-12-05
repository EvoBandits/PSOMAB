#ifndef _Arm_H_
#define _Arm_H_
#include <vector>
#include <random>


class Arm
{
private:
    std::vector<int> action_vector;
    int calc_cost_info{};
    double mu{}; 
    double sigma{};
    
    double r{};
    double k{};
    double Q{};
    


    
    
public:
    
    void reset_arm();
    double function_value(int obj, bool noise);
    double pull_arm(int obj);
    double get_Q();
    double get_k();
    double get_r();    
    double get_mu(); 

    std::vector<int> get_action_vector(); 


    void update_k(int k_var);
    void update_r(double r_var);
    void set_action_vector_element(int index, int value);
    void print_action_vector();

    
    Arm(std::vector<int> permutation,int calc_cost_info,         double init_r=0.0, double init_k=0.0, double init_Q=-1000000000.0);
    ~Arm();

};

#endif // _Arm_H_
