/*
 * @Author: Raiden49 
 * @Date: 2024-10-17 11:13:56 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:56:16
 */
#ifndef CILQR_CONSTRAINTS_INTERFACE_HPP_
#define CILQR_CONSTRAINTS_INTERFACE_HPP_

#include <casadi/casadi.hpp>
#include <Eigen/Dense>

namespace auto_drive {
namespace cilqr_planner{
class ConstraintsInterface {
 public:
  ConstraintsInterface(const casadi::DM& obs_list,
                       std::unordered_map<std::string, double>& params) : 
                       obs_list_(obs_list), params_(params) {
  };
  virtual ~ConstraintsInterface() = default;

  casadi::SX HardConstrain(const std::vector<casadi::SX>& cs, const double& eps = 1e-4) {
    casadi::SX cost = 0;
    for (const auto& c : cs) {
      // cost -= log(c + eps);
      cost += 1 / pow(c + eps, 2);
      // cost += 1 / pow(c + eps, 2) - log(c + eps);
    }
    return fmax(cost, 0);
  }
  casadi::SX Bounded(const casadi::SX& vars, const casadi::SX& uppers,
                     const casadi::SX& lowers, const double& eps = 1e-4) {
    std::vector<casadi::SX> cs;
    for (int i = 0; i < vars.size1(); i++) {
      casadi::SX diff = (uppers(i) - lowers(i)) / 2.0;
      cs.push_back((uppers(i) - vars(i)) / diff);
      cs.push_back((vars(i) - lowers(i)) / diff); 
    }
  return HardConstrain(cs, eps);             
  }
  casadi::SX SoftConstrain(const std::vector<casadi::SX>& cs, 
                           const double& alpha = 0.01, const double& beta = 10) {
    casadi::SX cost = 0;
    for (const auto& c : cs) {
      cost += alpha * exp(-beta * c);
    }
    return cost;
  }

 public:
  casadi::DM obs_list_;
  std::unordered_map<std::string, double>& params_;
};
} // namespace cilqr_planner
} // namespace auto_drive 

#endif //CILQR_CONSTRAINTS_INTERFACE_HPP_