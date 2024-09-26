/*
 * @Author: Raiden49 
 * @Date: 2024-09-18 20:01:56 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 15:06:51
 */
#include "plan.hpp"

namespace auto_drive {
void Plan::WayPointsPublish(const FrenetPath& final_path) {
  auto_drive::WaypointArray local_waypoints;
  local_waypoints.header.frame_id = "map";
  for (int i = 0; i < final_path.size_; i++) {
    Waypoint point;
    point.pose.pose.position.x = final_path.frenet_points[i].x;
    point.pose.pose.position.y = final_path.frenet_points[i].y;
    point.twist.twist.linear.x = final_path.frenet_points[i].v;
    local_waypoints.waypoints.push_back(point);
  }
  local_waypoints_pub_.publish(local_waypoints);
}
void Plan::Loop() {
  // is_planner_ = true;
  ros::Rate rate(10.0);
  
  int start_index = pre_match_index_;
  while (ros::ok()) {
    if (!common_info_ptr_->global_path_.empty() && 
        common_info_ptr_->is_odom_received_) {
      start_index = pre_match_index_;
      local_path_ = ref_line_ptr_->LocalPathTruncation(
          common_info_ptr_->cur_pose_, common_info_ptr_->global_path_, start_index);
      pre_match_index_ = ref_line_ptr_->match_index_;

      if (local_path_.size() > 1) {
        ref_path_ = ref_line_ptr_->LineSmooth(local_path_);

        CarState global_initial_point;
        if (is_first_loop_ || pre_final_path_.frenet_points.size() < 5) {
          global_initial_point = common_info_ptr_->cur_pose_;
          is_first_loop_ = false;
         }
        else if (fabs(common_info_ptr_->cur_pose_.x - pre_final_path_.frenet_points[0].x) > 2.0 ||
                 fabs(common_info_ptr_->cur_pose_.y - pre_final_path_.frenet_points[0].y) > 0.5) {
          double dt = 0.1;
          CarState cur_pose = common_info_ptr_->cur_pose_;
          CarState next_pose = cur_pose;

          double vx = cur_pose.vx * cos(cur_pose.yaw) - 
                      cur_pose.vy * sin(cur_pose.yaw);
          double vy = cur_pose.vx * sin(cur_pose.yaw) + 
                      cur_pose.vy * cos(cur_pose.yaw);
          double ax = cur_pose.ax * cos(cur_pose.yaw) - 
                      cur_pose.ay * sin(cur_pose.yaw);
          double ay = cur_pose.ax * sin(cur_pose.yaw) + 
                      cur_pose.ay * cos(cur_pose.yaw);
          next_pose.x = cur_pose.x + vx * dt + 0.5 * ax * dt * dt;
          next_pose.y = cur_pose.y + vy * dt + 0.5 * ay * dt * dt;
          next_pose.vx = cur_pose.vx + cur_pose.ax * dt;
          next_pose.vy = cur_pose.vy + cur_pose.ay * dt;
          next_pose.v = sqrt(pow(next_pose.vx, 2) + pow(next_pose.vy, 2));
          global_initial_point = next_pose;
        }
        else {
          global_initial_point = pre_final_path_.frenet_points[5];
        }
        auto initial_frenet_point = GetFrenetPoint(global_initial_point, 
                                                   ref_path_);
        auto collision_detection_ptr = std::make_shared<CollisionDetection>(
            common_info_ptr_->detected_objects_, collision_dis_, ref_path_);
        // Lattice Planner
        is_car_followed_ = false;
        FrenetPoint leader_car;
        for (auto& object : collision_detection_ptr->dynamic_obstacle_list_) {
          int frenet_match_index = SearchMatchIndex(object.point.x, 
                                                    object.point.y, 
                                                    ref_path_, 0);
          auto pro_point = GetProjectionPoint(object.point, 
                                              ref_path_[frenet_match_index]);
          auto frenet_obs = Cartesian2Frenet(object.point, pro_point);
          if (frenet_obs.s > initial_frenet_point.s && 
              fabs(frenet_obs.l - initial_frenet_point.l) < 2.0 &&
              fabs(frenet_obs.s - initial_frenet_point.s) < 3.0 * common_info_ptr_->cruise_speed_ && 
              object.point.v > 0.6 * common_info_ptr_->cruise_speed_) {
            is_car_followed_ = true;
            leader_car = frenet_obs;
            break;
          }
        }
        sample_paths_ = lattice_planner_ptr_->GetCandidatePaths(
            ref_path_, initial_frenet_point, leader_car, is_car_followed_);
        final_path_ = sample_paths_[0];
        pre_final_path_ = final_path_;
        history_paths_.push_back(final_path_);

        ROS_INFO("Find the best path!!! ,the size is%d", final_path_.size_);
        visualization_tool_ptr_->FinalPathVisualization(final_path_);
        visualization_tool_ptr_->SamplePathsVisualization(sample_paths_);
        visualization_tool_ptr_->
            ObjectSpeedVisualization(collision_detection_ptr->detected_objects_);
      }
    } 
    visualization_tool_ptr_->RefPathVisualization(ref_path_);
    visualization_tool_ptr_->HistoryPathVisualization(history_paths_);

    // publish path points to controller
    WayPointsPublish(final_path_);
    
    ros::spinOnce();
    rate.sleep();
  }
}
}