/*
 * @Author: Raiden49 
 * @Date: 2024-09-21 10:01:52 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-21 10:48:24
 */
#ifndef AUTO_DRIVE_VISUALIZATION_HPP_
#define AUTO_DRIVE_VISUALIZATION_HPP_

#include "m_common.hpp"

namespace auto_drive {
/**
 * @brief 在Rviz上可视化的类
 */
class VisualizationTool {
 public:
  VisualizationTool() = delete;
  VisualizationTool(ros::NodeHandle& nh) : nh_(nh) {

    // setup publishers
    ref_path_pub_ = nh_.advertise<
        nav_msgs::Path>("/reference_line/ref_path", 10);
    // sample_paths_pub_ = nh_.advertise<
    //     nav_msgs::Path>("/reference_line/sample_paths", 10);
    sample_paths_pub_ = nh_.advertise<
        visualization_msgs::MarkerArray>("/reference_line/sample_paths", 10);
    final_path_pub_ = nh_.advertise<
        nav_msgs::Path>("reference_line/final_path", 10);
    history_paths_pub_ = nh_.advertise<
        nav_msgs::Path>("/reference_line/history_paths", 10);
    speed_marker_pub_ = nh_.advertise<
        visualization_msgs::Marker>("/speed_marker_text", 10);
  }
  ~VisualizationTool() = default;

  void RefPathVisualization(const std::vector<PathPoint>& ref_path);
  void FinalPathVisualization(const FrenetPath& final_path);
  void SamplePathsVisualization(const std::vector<FrenetPath>& sample_paths);
  void HistoryPathVisualization(const std::vector<FrenetPath>& history_paths);
  void ObjectSpeedVisualization(const std::vector<Obstacle>& detected_objects);

 public:
  template<typename T>
  inline nav_msgs::Path TransformToNavMsgsPath(std::vector<T> points) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    for (auto& point : points) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(point.yaw);
      path.poses.push_back(pose);
    }
    return path;
  }

 public:
  ros::NodeHandle nh_;
  ros::Publisher local_waypoints_pub_;
  ros::Publisher ref_path_pub_;
  ros::Publisher sample_paths_pub_;
  ros::Publisher final_path_pub_;
  ros::Publisher history_paths_pub_;
  ros::Publisher speed_marker_pub_;
};
}

#endif // AUTO_DRIVE_VISUALIZATION_HPP_