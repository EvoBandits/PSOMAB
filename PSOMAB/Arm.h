#ifndef _Arm_H_
#define _Arm_H_
#include <Eigen/Core>
#include <random>
#include <vector>

class Arm {
private:
  Eigen::VectorXi action_vector;
  int calc_cost_info{};
  double mu{};
  double sigma{};

  double r{};
  double k{};
  double Q{};

public:
  void reset_arm();
  std::function<double(Eigen::VectorXi)> function_value;
  double pull_arm();
  double get_Q();
  double get_k();
  double get_r();
  double get_mu();

  Eigen::VectorXi get_action_vector();

  void update_k(int k_var);
  void update_r(double r_var);
  void set_action_vector_element(int index, int value);
  void print_action_vector();

  Arm(std::function<double(Eigen::VectorXi)> func, Eigen::VectorXi permutation, int calc_cost_info, double init_r = 0.0,
      double init_k = 0.0, double init_Q = -1000000000.0);
  ~Arm();
};

#endif // _Arm_H_
