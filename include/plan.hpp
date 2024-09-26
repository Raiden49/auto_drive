/*
 * @Author: Raiden49 
 * @Date: 2024-09-14 10:05:57 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 15:06:03
 */
#ifndef LATTICE_NODE_HPP_
#define LATTICE_NODE_HPP_

#include <memory> 
#include <unordered_map>
#include <ros/ros.h>

#include "visualization.hpp"
#include "reference_line.hpp"
#include "planner/lattice_planner.hpp"

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

    nh_.param<std::string>("role_name", role_name_, "ego_vehicle");
    nh_.param("local_length", local_length_, 50.0);
    nh_.param("is_planner", is_planner_, true);
    nh_.param("collision_dis", collision_dis_, 1.15);

    // lattice planner params
    nh_.param("plan/sample_max_time", lattice_params_["sample_max_time"], 4.0);
    nh_.param("plan/sample_min_time", lattice_params_["sample_min_time"], 2.0);
    nh_.param("plan/sample_time_step", lattice_params_["sample_time_step"], 0.5);
    nh_.param("plan/sample_lat_width", lattice_params_["sample_lat_width"], 3.5);
    nh_.param("plan/sample_width_length", 
              lattice_params_["sample_width_length"], 0.25);
    nh_.param("plan/weight_st_object", lattice_params_["weight_st_object"], 1.0);
    nh_.param("plan/weight_st_jerk", lattice_params_["weight_st_jerk"], 0.01);
    nh_.param("plan/weight_lt_offset", lattice_params_["weight_lt_offset"], 10.0);
    nh_.param("plan/weight_lt_acc", lattice_params_["weight_lt_acc"], 0.01);
  
    // reference line smoother params
    nh_.param("plan/ref_weight_smooth", ref_line_params_["ref_weight_smooth"], 70.0);
    nh_.param("plan/ref_weight_path_length", 
              ref_line_params_["ref_weight_path_length"], 10.0);
    nh_.param("plan/ref_weight_ref_deviation", 
              ref_line_params_["ref_weight_ref_deviation"], 20.0);
    nh_.param("plan/lower_bound", ref_line_params_["lower_bound"], -2.0);
    nh_.param("plan/upper_bound", ref_line_params_["upper_bound"], 2.0);

    common_info_ptr_ = std::make_shared<CommonInfo>(role_name_, nh);
    visualization_tool_ptr_ = std::make_shared<VisualizationTool>(nh);
    ref_line_ptr_ = std::make_shared<ReferenceLine>(local_length_,
                                                    ref_line_params_);
    collision_detection_ptr_ = std::make_shared<CollisionDetection>(
        common_info_ptr_->detected_objects_, collision_dis_, ref_path_);
    lattice_planner_ptr_ = std::make_shared<planner::LatticePlanner>(
        common_info_ptr_->cruise_speed_, lattice_params_, collision_detection_ptr_);
    local_waypoints_pub_ = nh_.advertise<
        auto_drive::WaypointArray>("/reference_line/local_waypoint", 10);
  }
  ~Plan() = default;
  void Loop();

  void WayPointsPublish(const FrenetPath& final_path);

 public:
  std::shared_ptr<CommonInfo> common_info_ptr_;
  std::shared_ptr<VisualizationTool> visualization_tool_ptr_;
  std::shared_ptr<ReferenceLine> ref_line_ptr_;
  std::shared_ptr<CollisionDetection> collision_detection_ptr_;
  std::shared_ptr<planner::LatticePlanner> lattice_planner_ptr_;

 private:
  bool is_planner_, is_first_loop_, is_car_followed_;
  double local_length_;
  int pre_match_index_;
  std::vector<PathPoint> local_path_;
  std::vector<PathPoint> ref_path_;
  FrenetPath final_path_;
  FrenetPath pre_final_path_;
  std::vector<FrenetPath> sample_paths_;
  std::vector<FrenetPath> history_paths_;
  ros::Publisher local_waypoints_pub_;

 private:
  double collision_dis_;
  std::string role_name_;
  ros::NodeHandle nh_;
  std::unordered_map<std::string, double> lattice_params_;
  std::unordered_map<std::string, double> ref_line_params_;
  std::unordered_map<std::string, double> dp_path_params_;
  std::unordered_map<std::string, double> qp_path_params_;
};  
}

#endif // LATTICE_NODE_HPP_