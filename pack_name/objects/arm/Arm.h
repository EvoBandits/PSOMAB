#ifndef _Arm_H_
#define _Arm_H_
#include "Eigen/Core"
#include <functional>
#include <random>
#include <vector>

class Arm {
       public:
        // ToDo: rename permutation to action_vector, init_r to reward, init_k to num_pulls, init_Q to reward
        Arm(std::function<double(Eigen::VectorXi, int)> func, Eigen::VectorXi permutation, double init_r = 0.0,
            int init_k = 0.0, double init_Q = -1000000000.0);
        double function_value() const;
        double pull();
        double get_Q() const;
        int num_pulls() const;
        double reward() const;
        Eigen::VectorXi get_action_vector() const;
        void update_num_pulls(int k_var);
        void update_reward(double r_var);
        double mean_reward() const;
        double true_value() const;

       private:
        Eigen::VectorXi action_vector_;
        double reward_{};
        int num_pulls_{};
        double Q_{};
        // ToDo: check if pointer to function is better
        std::function<double(Eigen::VectorXi, int)> arm_func_;
};

#endif// _Arm_H_
