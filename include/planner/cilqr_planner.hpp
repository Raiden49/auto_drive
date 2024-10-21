/*
 * @Author: Raiden49 
 * @Date: 2024-10-16 09:34:17 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:09:10
 */
#ifndef CILQR_PLANNER_HPP_
#define CILQR_PLANNER_HPP_

#include "planner/cilqr.hpp"
#include "collision_detection.hpp"

namespace auto_drive {
namespace cilqr_planner {
class CiLQRPlanner {
 public:
  CiLQRPlanner(std::unordered_map<std::string, double>& params, 
               std::shared_ptr<CollisionDetection> collision_detection_ptr) : 
               params_(params), collision_detection_ptr_(collision_detection_ptr) {
    ParamsPrint();
  };
  ~CiLQRPlanner() = default;

  FrenetPath Solve(const std::vector<PathPoint>& ref_path, 
                   const FrenetPoint& initial_frenet_point,
                   const FrenetPoint& goal_frenet_point,
                   const double& vehicle_heading);
  void ParamsPrint() {
    ROS_INFO("Check cilqr params: ");
    ROS_INFO_STREAM("delta_t: " << params_["delta_t"]);
    ROS_INFO_STREAM("horizon: " << params_["horizon"]);
    ROS_INFO_STREAM("num_state: " << params_["num_state"]);
    ROS_INFO_STREAM("num_control: " << params_["num_control"]);
    ROS_INFO_STREAM("w_acc: " << params_["w_acc"]);
    ROS_INFO_STREAM("w_yaw_rate: " << params_["w_yaw_rate"]);
    ROS_INFO_STREAM("w_pos_s: " << params_["w_pos_s"]);
    ROS_INFO_STREAM("w_pos_l: " << params_["w_pos_l"]);
    ROS_INFO_STREAM("w_vel: " << params_["w_vel"]);
    ROS_INFO_STREAM("q1_acc: " << params_["q1_acc"]);
    ROS_INFO_STREAM("q2_acc: " << params_["q2_acc"]);
    ROS_INFO_STREAM("q1_yaw_rate: " << params_["q1_yaw_rate"]);
    ROS_INFO_STREAM("q2_yaw_rate: " << params_["q2_yaw_rate"]);
    ROS_INFO_STREAM("q1_front: " << params_["q1_front"]);
    ROS_INFO_STREAM("q2_front: " << params_["q2_front"]);
    ROS_INFO_STREAM("q1_rear: " << params_["q1_rear"]);
    ROS_INFO_STREAM("q2_rear: " << params_["q2_rear"]);
    ROS_INFO_STREAM("acc_min: " << params_["acc_min"]);
    ROS_INFO_STREAM("acc_max: " << params_["acc_max"]);
    ROS_INFO_STREAM("steer_min: " << params_["steer_min"]);
    ROS_INFO_STREAM("steer_max: " << params_["steer_max"]);
    ROS_INFO_STREAM("wheel_base: " << params_["wheel_base"]);
    ROS_INFO_STREAM("max_speed: " << params_["max_speed"]);
    ROS_INFO_STREAM("t_safe: " << params_["t_safe"]);
    ROS_INFO_STREAM("s_safe_length: " << params_["s_safe_length"]);
    ROS_INFO_STREAM("s_safe_width: " << params_["s_safe_width"]);
    ROS_INFO_STREAM("car_length: " << params_["car_length"]);
    ROS_INFO_STREAM("car_width: " << params_["car_width"]);
    ROS_INFO_STREAM("ego_rad: " << params_["ego_rad"]);
    ROS_INFO_STREAM("ego_lf: " << params_["ego_lf"]);
    ROS_INFO_STREAM("ego_lr: " << params_["ego_lr"]);
    ROS_INFO_STREAM("max_iters: " << params_["max_iters"]);
    ROS_INFO_STREAM("tol: " << params_["tol"]);
    ROS_INFO_STREAM("desired_speed: " << params_["desired_speed"]);
  };

 public:
  std::vector<FrenetPath> all_possible_trajs_;
  std::unordered_map<std::string, double>& params_;
  std::shared_ptr<CollisionDetection> collision_detection_ptr_;
};
} // namespace cilqr_planner
} // namespace auto_drive

#endif // CILQR_PLANNER_HPP_