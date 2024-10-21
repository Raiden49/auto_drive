/*
 * @Author: Raiden49 
 * @Date: 2024-10-17 10:33:32 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:47:22
 */
#ifndef CILQR_CONSTRAINTS_HPP_
#define CILQR_CONSTRAINTS_HPP_

#include "planner/cilqr_constraints_interface.hpp"

namespace auto_drive {
namespace cilqr_planner{
class Constraints : public ConstraintsInterface {
 public:
  Constraints(const casadi::DM& obs_list,
              std::unordered_map<std::string, double>& params) : 
              ConstraintsInterface(obs_list, params) {
  };
  ~Constraints() = default;

  casadi::SX StateCostConstruct(const casadi::SX& state);
  casadi::SX ControlCostConstruct(const casadi::SX& control);
  casadi::SX ObsCostConstruct(const casadi::SX& state);
  casadi::SX CostConstruct(const casadi::SX& state, const casadi::SX& control);
  std::tuple<casadi::SX, casadi::SX, casadi::SX> CostDebug(const casadi::SX& state, 
                                                           const casadi::SX& control);
};
} // namespace cilqr_planner
} // namespace auto_drive 


#endif // CILQR_CONSTRAINTS_HPP_