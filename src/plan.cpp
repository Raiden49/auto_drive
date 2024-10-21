/*
 * @Author: Raiden49 
 * @Date: 2024-09-18 20:01:56 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-16 12:49:53
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
void Plan::ControllerSim(const FrenetPath& final_path) {
  for (int i = 0; i < std::min(10, final_path.size_); i++) {
    geometry_msgs::Pose transform_pose;
    transform_pose.position.x = final_path.frenet_points[i].x;
    transform_pose.position.y = final_path.frenet_points[i].y;
    transform_pose.orientation = 
        tf::createQuaternionMsgFromYaw(final_path.frenet_points[1].yaw);
    controller_sim_pub_.publish(transform_pose);
}
}
void Plan::Loop() {
  ros::Rate rate(100.0);
  
  int start_index = pre_match_index_;
  while (ros::ok()) {
    if (!common_info_ptr_->global_path_.empty() && 
        common_info_ptr_->is_odom_received_) {
      auto&& optim_global_path = 
          ref_line_ptr_->LineSmooth(common_info_ptr_->global_path_);

      start_index = pre_match_index_;
      local_path_ = ref_line_ptr_->LocalPathTruncation(
          common_info_ptr_->cur_pose_, optim_global_path, start_index);
      pre_match_index_ = ref_line_ptr_->match_index_;

      ref_path_ = local_path_;

      if (local_path_.size() > 1) {

        collision_detection_ptr_ = std::make_shared<CollisionDetection>(
            common_info_ptr_->detected_objects_, collision_dis_, ref_path_);

        CarState global_initial_point;
        if (is_first_loop_ || pre_final_path_.frenet_points.size() < 5) {
          global_initial_point = common_info_ptr_->cur_pose_;
          is_first_loop_ = false;
         }
        else if (EuclideanDis(common_info_ptr_->cur_pose_.x, 
                              common_info_ptr_->cur_pose_.y,
                              pre_final_path_.frenet_points[0].x, 
                              pre_final_path_.frenet_points[0].y) > 100) {
        // else if (fabs(common_info_ptr_->cur_pose_.x - pre_final_path_.frenet_points[0].x) > 2.0 ||
                //  fabs(common_info_ptr_->cur_pose_.y - pre_final_path_.frenet_points[0].y) > 0.5) {
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
          global_initial_point = pre_final_path_.frenet_points[3];
        }

        global_initial_point.x = common_info_ptr_->cur_pose_.x;
        global_initial_point.y = common_info_ptr_->cur_pose_.y;
        global_initial_point.v = common_info_ptr_->cur_pose_.v;

        auto initial_frenet_point = Cartesian2Frenet(global_initial_point, 
                                                     ref_path_);
        CarState global_goal_point;
        global_goal_point.x = ref_path_[ref_path_.size() - 5].x;
        global_goal_point.y = ref_path_[ref_path_.size() - 5].y;
        global_goal_point.v = 4;
        auto goal_frenet_point = Cartesian2Frenet(global_goal_point, ref_path_);

        // std::cout << "-------------------initial debug-------------------" << std::endl;
        // std::cout << "global initial point: " << global_initial_point.x << ", "
        //           << global_initial_point.y << std::endl;
        // std::cout << "initial frenet point: " << initial_frenet_point.s << ", "
        //           << initial_frenet_point.l << ", " << initial_frenet_point.yaw
        //           << std::endl;
        
        auto collision_detection_ptr = std::make_shared<CollisionDetection>(
            common_info_ptr_->detected_objects_, collision_dis_, ref_path_);
        // Lattice Planner
        is_car_followed_ = false;
        FrenetPoint leader_car;
        for (auto& object : collision_detection_ptr->dynamic_obstacle_list_) {
          // std::cout << "test: " << object.point.x << ", " << object.point.y << std::endl; 
          auto frenet_obs = Cartesian2Frenet(object.point, ref_path_);
          if (frenet_obs.s > initial_frenet_point.s && 
              fabs(frenet_obs.l - initial_frenet_point.l) < 2.0 &&
              fabs(frenet_obs.s - initial_frenet_point.s) < 3.0 * common_info_ptr_->cruise_speed_ && 
              object.point.v > 0.6 * common_info_ptr_->cruise_speed_) {
            is_car_followed_ = true;
            leader_car = frenet_obs;
            break;
          }
        }

        clock_t start_time = clock();

        if (planner_method_ == "lattice_planner") {
          lattice_planner_ptr_->collision_detection_ptr_ = collision_detection_ptr;
          sample_paths_ = lattice_planner_ptr_->GetCandidatePaths(
              ref_path_, initial_frenet_point, leader_car, is_car_followed_);
          
          if (sample_paths_.size() == 0) {
            ROS_ERROR("Lattice planner can't find any valid path");
          }
          final_path_ = sample_paths_[0];
          visualization_tool_ptr_->SamplePathsVisualization(sample_paths_);
        }
        else if (planner_method_ == "em_planner") {
          em_planner_ptr_->lane_left_l_ = 0.75 * common_info_ptr_->lane_width_;
          em_planner_ptr_->lane_right_l_ = 0 - 0.25 * common_info_ptr_->lane_width_;

          em_planner_ptr_->collision_detection_ptr_ = collision_detection_ptr;
          final_path_ = em_planner_ptr_->Planning(ref_path_, initial_frenet_point);
          sample_paths_ = em_planner_ptr_->GetSamplePath(ref_path_);
          // visualization_tool_ptr_->SamplePathsVisualization(sample_paths_);
        }
        else if (planner_method_ == "cilqr_planner") {
          cilqr_planner_ptr_->collision_detection_ptr_ = collision_detection_ptr;
          final_path_ = cilqr_planner_ptr_->Solve(ref_path_, initial_frenet_point,
                                                  goal_frenet_point, 
                                                  common_info_ptr_->cur_pose_.yaw);
          visualization_tool_ptr_->SamplePathsVisualization(cilqr_planner_ptr_->all_possible_trajs_);
        }
        else {
          ROS_ERROR("Please set a planner!!!!!!");
        }

        clock_t end_time = clock();
        double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;

        pre_final_path_ = final_path_;
        history_paths_.push_back(final_path_);

        // std::vector<PathPoint> cartesian_final_path;
        // cartesian_final_path.resize(final_path_.frenet_points.size());
        // for (int i = 0; i < final_path_.frenet_points.size(); i++) {
        //   cartesian_final_path[i].x = final_path_.frenet_points[i].x;
        //   cartesian_final_path[i].y = final_path_.frenet_points[i].y;
        // }
        // qp_optimer_ptr_ = std::make_shared<optimization::QP>(
        //     -0.5, 0.5, 10, 1, 1, cartesian_final_path);
        // auto&& optim_points = qp_optimer_ptr_->Process();
        // for (int i = 0; i < optim_points.size(); i++) {
        //   final_path_.frenet_points[i].x = optim_points[i].x;
        //   final_path_.frenet_points[i].y = optim_points[i].y;
        // }

        ROS_INFO("Find the best path!!! ,the size is %d, %f ms time used", final_path_.size_, run_time);
        visualization_tool_ptr_->FinalPathVisualization(final_path_);
        visualization_tool_ptr_->
            ObjectSpeedVisualization(collision_detection_ptr->detected_objects_);
        // ControllerSim(final_path_);
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