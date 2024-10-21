/*
 * @Author: Raiden49 
 * @Date: 2024-10-17 13:26:40 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:57:30
 */
#include "planner/cilqr_constraints.hpp"
namespace auto_drive {
namespace cilqr_planner{
casadi::SX Constraints::StateCostConstruct(const casadi::SX& state) {
  // auto state_cost_weight = casadi::SX::zeros(params_["num_state"], params_["num_state"]);
  // state_cost_weight(0, 0) = params_["w_pos_s"];
  // state_cost_weight(1, 1) = params_["w_pos_l"];
  // state_cost_weight(2, 2) = params_["w_vel"];
  // state_cost_weight(3, 3) = 0;
  // auto&& target_state = casadi::SX::vertcat({20, 0, params_["desire_speed"], 0});
  // auto&& state_diff = state - target_state;
  // auto state_cost = mtimes(mtimes(state_diff.T(), state_cost_weight), state_diff);

  // casadi::SX vars = casadi::SX::vertcat({state(1), state(3)});
  // casadi::SX uppers = casadi::SX::vertcat({5, M_PI});
  // casadi::SX lowers = casadi::SX::vertcat({-2, -M_PI});
  // state_cost += Bounded(vars, uppers, lowers);

  auto state_cost = pow(state(1), 2);
  state_cost += pow(state(2) - params_["desired_speed"], 2);
  
  return state_cost;
}
casadi::SX Constraints::ControlCostConstruct(const casadi::SX& control) {
  // auto control_cost_weight = casadi::SX::zeros(params_["num_control"], 
  //                                              params_["num_control"]);
  // control_cost_weight(0, 0) = params_["w_acc"];
  // control_cost_weight(1, 1) = params_["w_yaw_rate"];
  // auto control_cost = mtimes(mtimes(control.T(), control_cost_weight), control);

  // casadi::SX vars = casadi::SX::vertcat({control(0), control(1)});
  // casadi::SX uppers = casadi::SX::vertcat({5, 2});
  // casadi::SX lowers = casadi::SX::vertcat({-5, -2});
  // control_cost += Bounded(vars, uppers, lowers);

  auto control_cost = 0.1 * pow(control(0), 2) + 0.1 * pow(control(1), 2);
  
  return control_cost; 
}
casadi::SX Constraints::ObsCostConstruct(const casadi::SX& state) {
  casadi::SX obs_cost = 0;
  for (int i = 0; i < obs_list_.size2(); i++) {
    double obs_s = obs_list_(0, i).scalar();
    double obs_l = obs_list_(1, i).scalar();
    double obs_v = obs_list_(2, i).scalar();
    double obs_yaw = obs_list_(3, i).scalar();

    double elliptical_a = params_["car_length"] + params_["s_safe_length"] + 
                          params_["ego_rad"] + abs(obs_v * cos(obs_yaw)) * params_["t_safe"];
    double elliptical_b = params_["car_width"] + params_["s_safe_width"] + 
                          params_["ego_rad"] + abs(obs_v * sin(obs_yaw)) * params_["t_safe"];
    

    // auto transform_matrix = casadi::SX::zeros(params_["num_state"], params_["num_state"]);
    // transform_matrix(0, 0) = cos(obs_yaw);
    // transform_matrix(0, 1) = sin(obs_yaw);
    // transform_matrix(1, 0) = -sin(obs_yaw);
    // transform_matrix(1, 1) = cos(obs_yaw);

    casadi::SX collision_zone = pow(state(0) - obs_s, 2) / pow(elliptical_a, 2) + 
                                pow(state(1) - obs_l, 2) / pow(elliptical_b, 2) - 1;
    obs_cost += HardConstrain(std::vector<casadi::SX>{collision_zone});

    // obs_cost += SoftConstrain(std::vector<casadi::SX>{collision_zone});
  }

  return obs_cost;
}
casadi::SX Constraints::CostConstruct(const casadi::SX& state, 
                                      const casadi::SX& control) {
  return StateCostConstruct(state) + ControlCostConstruct(control) + ObsCostConstruct(state);
}
std::tuple<casadi::SX, casadi::SX, casadi::SX> Constraints::CostDebug(
    const casadi::SX& state, const casadi::SX& control) {
  return {StateCostConstruct(state), ControlCostConstruct(control), ObsCostConstruct(state)};
}

} // namespace cilqr_planner
} // namespace auto_drive 