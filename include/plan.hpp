/*
 * @Author: Raiden49 
 * @Date: 2024-09-14 10:05:57 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 15:06:03
 */
#ifndef AUTO_DRIVE_PLAN_HPP_
#define AUTO_DRIVE_PLAN_HPP_

#include <memory> 
#include <unordered_map>
#include <ros/ros.h>

#include "visualization.hpp"
#include "reference_line.hpp"
#include "planner/lattice_planner.hpp"
#include "planner/em_planner.hpp"
#include "optim/qp_optim.hpp"

#include "auto_drive/Waypoint.h"
#include "auto_drive/WaypointArray.h"

namespace auto_drive {
class Plan {
 public:     
  Plan() = delete;
  Plan(ros::NodeHandle& nh) : nh_(nh) {
    pre_match_index_ = 0;
    is_car_followed_ = false;
    is_first_loop_ = false;
    collision_dis_ = 1.0;

    ros::param::get("plan/role_name", role_name_);
    ros::param::get("plan/local_length", local_length_);
    ros::param::get("plan/collision_dis", collision_dis_);
    ros::param::get("plan/method", planner_method_);
    
    // lattice planner params
    ros::param::get("plan/lattice_planner/sample_max_time", 
              lattice_params_["sample_max_time"]);
    ros::param::get("plan/lattice_planner/sample_min_time", 
              lattice_params_["sample_min_time"]);
    ros::param::get("plan/lattice_planner/sample_time_step", 
              lattice_params_["sample_time_step"]);
    ros::param::get("plan/lattice_planner/sample_lat_width", 
              lattice_params_["sample_lat_width"]);
    ros::param::get("plan/lattice_planner/sample_width_length", 
              lattice_params_["sample_width_length"]);
    ros::param::get("plan/lattice_planner/weight_st_object", 
              lattice_params_["weight_st_object"]);
    ros::param::get("plan/lattice_planner/weight_st_jerk", 
              lattice_params_["weight_st_jerk"]);
    ros::param::get("plan/lattice_planner/weight_lt_offset", 
              lattice_params_["weight_lt_offset"]);
    ros::param::get("plan/lattice_planner/weight_lt_acc", 
              lattice_params_["weight_lt_acc"]);
  
    // reference line smoother params
    ros::param::get("plan/ref_weight_smooth", ref_line_params_["ref_weight_smooth"]);
    ros::param::get("plan/ref_weight_path_length", 
              ref_line_params_["ref_weight_path_length"]);
    ros::param::get("plan/ref_weight_ref_deviation", 
              ref_line_params_["ref_weight_ref_deviation"]);
    ros::param::get("plan/lower_bound", ref_line_params_["lower_bound"]);
    ros::param::get("plan/upper_bound", ref_line_params_["upper_bound"]);

    // dp path params
    ros::param::get("plan/em_planner/dp_sample_l", dp_path_params_["dp_sample_l"]);
    ros::param::get("plan/em_planner/dp_sample_s", dp_path_params_["dp_sample_s"]);
    ros::param::get("plan/em_planner/dp_sample_rows", dp_path_params_["dp_sample_rows"]);
    ros::param::get("plan/em_planner/dp_sample_cols", dp_path_params_["dp_sample_cols"]);
    ros::param::get("plan/em_planner/dp_cost_collision", dp_path_params_["dp_cost_collision"]);
    ros::param::get("plan/em_planner/dp_cost_dl", dp_path_params_["dp_cost_dl"]);
    ros::param::get("plan/em_planner/dp_cost_ddl", dp_path_params_["dp_cost_ddl"]);
    ros::param::get("plan/em_planner/dp_cost_dddl", dp_path_params_["dp_cost_dddl"]);
    ros::param::get("plan/em_planner/dp_cost_ref", dp_path_params_["dp_cost_ref"]);
    // qp path params
    ros::param::get("plan/em_planner/qp_cost_l", qp_path_params_["qp_cost_l"]);
    ros::param::get("plan/em_planner/qp_cost_dl", qp_path_params_["qp_cost_dl"]);
    ros::param::get("plan/em_planner/qp_cost_ddl", qp_path_params_["qp_cost_ddl"]);
    ros::param::get("plan/em_planner/qp_cost_dddl", qp_path_params_["qp_cost_dddl"]);
    ros::param::get("plan/em_planner/qp_cost_ref", qp_path_params_["qp_cost_ref"]);
    ros::param::get("plan/em_planner/qp_cost_end_l", qp_path_params_["qp_cost_end_l"]);
    ros::param::get("plan/em_planner/qp_cost_end_dl", qp_path_params_["qp_cost_end_dl"]);
    ros::param::get("plan/em_planner/qp_cost_end_ddl", qp_path_params_["qp_cost_end_ddl"]);
    
    common_info_ptr_ = std::make_shared<CommonInfo>(role_name_, nh);
    visualization_tool_ptr_ = std::make_shared<VisualizationTool>(nh);
    ref_line_ptr_ = std::make_shared<ReferenceLine>(local_length_,
                                                    ref_line_params_);
    lattice_planner_ptr_ = std::make_shared<planner::LatticePlanner>(
        common_info_ptr_->cruise_speed_, lattice_params_, collision_detection_ptr_);
    em_planner_ptr_ = std::make_shared<planner::EMPlanner>(
        dp_path_params_, qp_path_params_, collision_detection_ptr_);
    local_waypoints_pub_ = nh_.advertise<
        auto_drive::WaypointArray>("/reference_line/local_waypoint", 10);
    controller_sim_pub_ = nh.advertise<
        geometry_msgs::Pose>("/carla/ego_vehicle/control/set_transform", 10);
  }
  ~Plan() = default;
  void Loop();

  void ControllerSim(const FrenetPath& final_path);
  void WayPointsPublish(const FrenetPath& final_path);

 public:
  std::shared_ptr<CommonInfo> common_info_ptr_;
  std::shared_ptr<VisualizationTool> visualization_tool_ptr_;
  std::shared_ptr<ReferenceLine> ref_line_ptr_;
  std::shared_ptr<CollisionDetection> collision_detection_ptr_;
  std::shared_ptr<planner::LatticePlanner> lattice_planner_ptr_;
  std::shared_ptr<planner::EMPlanner> em_planner_ptr_;
  std::shared_ptr<optimization::QP> qp_optimer_ptr_;

 private:
  bool is_first_loop_;
  bool is_car_followed_;                // 判断是否正在跟车
  double local_length_;                 // 截取的参考线长度
  int pre_match_index_;                 // 上一周期匹配的index
  std::vector<PathPoint> local_path_;   // 局部参考路径
  std::vector<PathPoint> ref_path_;     // 参考路径
  FrenetPath final_path_;               // 最终要跟踪的路径
  FrenetPath pre_final_path_;           // 上一周期要跟踪的路径
  std::vector<FrenetPath> sample_paths_;// 所有采样到的路径
  std::vector<FrenetPath> history_paths_;
  ros::Publisher local_waypoints_pub_;  // 发送给控制器的消息
  ros::Publisher controller_sim_pub_;

 private:
  double collision_dis_;                // 碰撞距离
  std::string planner_method_;
  std::string role_name_;
  ros::NodeHandle nh_;
  std::unordered_map<std::string, double> lattice_params_;
  std::unordered_map<std::string, double> ref_line_params_;
  std::unordered_map<std::string, double> dp_path_params_;
  std::unordered_map<std::string, double> qp_path_params_;
};  
}

#endif // AUTO_DRIVE_PLAN_HPP_