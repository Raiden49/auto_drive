/*
 * @Author: Raiden49 
 * @Date: 2024-10-16 09:48:41 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:26:55
 */

#include "planner/cilqr_planner.hpp"

namespace auto_drive {
namespace cilqr_planner {
void GetCartesianPaths(FrenetPath& frenet_path, 
                       const std::vector<PathPoint>& ref_path) {
  frenet_path.size_ = 0;
  for (int i = 0; i < frenet_path.frenet_points.size(); i++) {
    if (frenet_path.frenet_points[i].s >= ref_path.back().s_) {
      break;
    }
    frenet_path.size_++;
  }
  for (int i = 0; i< frenet_path.size_; i++) {
    Frenet2Cartesian(frenet_path.frenet_points[i], ref_path);
  }
}
FrenetPath CiLQRPlanner::Solve(const std::vector<PathPoint>& ref_path, 
                               const FrenetPoint& initial_frenet_point,
                               const FrenetPoint& goal_frenet_point,
                               const double& vehicle_heading) {   
  int static_obs_num = collision_detection_ptr_->static_obstacle_list_.size();   
  casadi::DM static_obs_list = casadi::DM::zeros(params_["num_state"], static_obs_num);
  for (int i = 0; i < static_obs_num; i++) {
    static_obs_list(0, i) = collision_detection_ptr_->static_obstacle_list_[i].point.s;
    static_obs_list(1, i) = collision_detection_ptr_->static_obstacle_list_[i].point.l;
    static_obs_list(2, i) = collision_detection_ptr_->static_obstacle_list_[i].point.v;
    static_obs_list(3, i) = collision_detection_ptr_->static_obstacle_list_[i].point.yaw;
  }
  auto cilqr_ptr_ = std::make_shared<CiLQR>(static_obs_list, params_);
  
  casadi::DM state_init = casadi::DM::vertcat({initial_frenet_point.s, 
                                               initial_frenet_point.l, 
                                               initial_frenet_point.v, 
                                               vehicle_heading});
  casadi::DM control_init = casadi::DM::zeros(params_["num_control"], 
                                              params_["horizon"]);                                 
  auto [states, controls] = cilqr_ptr_->Loop(state_init, control_init);

  auto all_possible_trajs = cilqr_ptr_->return_all_possible_trajs_();
  for (int i = 0; i < all_possible_trajs.size(); i++) {
    FrenetPath frenet_path;
    for (int n = 0; n < all_possible_trajs[i].size2(); n++) {
      FrenetPoint point;
      point.s = all_possible_trajs[i](0, n).scalar();
      point.l = all_possible_trajs[i](1, n).scalar();
      point.v = all_possible_trajs[i](2, n).scalar();
      point.yaw = all_possible_trajs[i](3, n).scalar();
      frenet_path.frenet_points.push_back(point);
    }
    GetCartesianPaths(frenet_path, ref_path);
    all_possible_trajs_.push_back(frenet_path);
  }

  FrenetPath frenet_path;
  for (int i = 0; i < states.size2(); i++) {
    FrenetPoint point;
    point.s = states(0, i).scalar();
    point.l = states(1, i).scalar();
    point.v =  4;
    point.yaw = states(3, i).scalar();
    frenet_path.frenet_points.push_back(point);
    
    // std::cout << "-------------------state debug-------------------" << std::endl;
    // std::cout << "state s: " << states(0, i).scalar() << std::endl;
    // std::cout << "state l: " << states(1, i).scalar() << std::endl;
    // std::cout << "state v: " << states(2, i).scalar() << std::endl;
    // std::cout << "state yaw: " << states(3, i).scalar() << std::endl;
  }
  GetCartesianPaths(frenet_path, ref_path);
  return frenet_path;
}
} // namespace cilqr_planner
} // namespace auto_drive
