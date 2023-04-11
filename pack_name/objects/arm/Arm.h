#ifndef _Arm_H_
#define _Arm_H_
#include "Eigen/Core"
#include <functional>
#include <random>
#include <vector>

class Arm {
       public:
        Arm(std::function<double(Eigen::VectorXi, int)> func, Eigen::VectorXi action_vector, double initial_reward = 0.0, int initial_num_pulls = 0);
        double function_value() const;
        double pull();
        void pull(int num_pulls);
        int num_pulls() const;
        double reward() const;
        Eigen::VectorXi get_action_vector() const;
        void update_num_pulls(int new_num_pulls);
        void update_reward(double new_reward);
        double mean_reward() const;
        double variance() const;
        double true_value() const;

       private:
        Eigen::VectorXi action_vector_;
        double reward_ = 0;
        double reward_squared_ = 0;
        int num_pulls_;
        // ToDo: check if pointer to function is better
        std::function<double(Eigen::VectorXi, int)> arm_func_;
};

#endif// _Arm_H_
