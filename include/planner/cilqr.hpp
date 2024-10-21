/*
 * @Author: Raiden49 
 * @Date: 2024-10-12 14:40:08 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:51:01
 */
#ifndef CILQR_HPP_
#define CILQR_HPP_

// #include "planner/cilqr_containers.hpp"
#include "planner/cilqr_constraints.hpp"
#include "planner/cilqr_vehicle_model.hpp"

namespace auto_drive {
namespace cilqr_planner {
class CiLQR {
 public:
  CiLQR(const casadi::DM& obs_list,
        std::unordered_map<std::string, double>& params) : params_(params) {
    constraints_ptr_ = std::make_shared<Constraints>(obs_list, params);
    vehicle_model_ptr_ = std::make_shared<VehicleModel>(params);
  }
  ~CiLQR() = default;

  bool SolverInit(casadi::SX& state, casadi::SX& control);

  std::tuple<casadi::DM, casadi::DM> Loop(const casadi::DM& x0, 
                                          const casadi::DM& us_init);
  casadi::DM RollOut(const casadi::DM& x0, const casadi::DM& us);
  std::tuple<casadi::DM, casadi::DM, double> ForwardPass(const double& alpha,
                                                         const casadi::DM& xs, 
                                                         const casadi::DM& us, 
                                                         const casadi::DM& ks, 
                                                         const std::vector<casadi::DM>& Ks);
  std::tuple<casadi::DM, std::vector<casadi::DM>, double> BackwardPass(
      const casadi::DM& xs, const casadi::DM& us, double regu);
  std::vector<casadi::DM> return_all_possible_trajs_() {
    return this->all_possible_trajs_;
  };

 public:
  double lamb_factor_ = 10;
  double max_lamb_ = 1000;
  std::unordered_map<std::string, double>& params_;
  std::shared_ptr<Constraints> constraints_ptr_;
  std::shared_ptr<VehicleModel> vehicle_model_ptr_;
  std::vector<double> alphas = {0.5, 0.25, 0.125, 0.0625};

 private:
  std::vector<casadi::DM> all_possible_trajs_;
  casadi::Function f_, f_x_, f_u_;
  casadi::Function l_, l_debug_, l_x_, l_xx_, l_u_, l_ux_, l_uu_;
};
} // namespace cilqr_planner
} // namespace auto_drive

#endif // CILQR_HPP_