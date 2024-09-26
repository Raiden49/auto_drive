/*
 * @Author: Raiden49 
 * @Date: 2024-09-21 10:04:04 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-21 10:44:32
 */
#include "visualization.hpp"

namespace auto_drive {
void VisualizationTool::RefPathVisualization(const std::vector<PathPoint>& ref_path) {
  auto&& path = TransformToNavMsgsPath(ref_path);
  ref_path_pub_.publish(path);
}
void VisualizationTool::FinalPathVisualization(const FrenetPath& final_path) {
  auto&& path = TransformToNavMsgsPath(final_path.frenet_points);
  final_path_pub_.publish(path);
}
void VisualizationTool::SamplePathsVisualization(
    const std::vector<FrenetPath>& sample_paths) {
  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < sample_paths.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sample_path";
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.r = 0.3;
    marker.color.g = 0.5;
    marker.color.b = 1;
    marker.color.a = 0.3;
    for (auto& point : sample_paths[i].frenet_points) {
      geometry_msgs::Point vtx;
      vtx.x = point.x;
      vtx.y = point.y;
      marker.points.push_back(vtx);
    }
    marker_array.markers.push_back(marker);
  }
  sample_paths_pub_.publish(marker_array);
}
void VisualizationTool::HistoryPathVisualization(
    const std::vector<FrenetPath>& history_paths) {
  for (auto& history_path : history_paths) {
    auto&& path = TransformToNavMsgsPath(history_path.frenet_points);
    history_paths_pub_.publish(path);
  }
}
void VisualizationTool::ObjectSpeedVisualization(
    const std::vector<Obstacle>& detected_objects) {
  int id_ = 0;
  for (auto& object : detected_objects) {
    visualization_msgs::Marker speed_marker;
    speed_marker.header.frame_id = "map";
    speed_marker.header.stamp = ros::Time::now();
    speed_marker.ns = "auto_drive/speed_marker";
    speed_marker.action = visualization_msgs::Marker::ADD;
    speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    speed_marker.pose.orientation.w = 1.0;
    speed_marker.id = id_++;
    speed_marker.scale.x = 1.5;
    speed_marker.scale.y = 1.5;
    speed_marker.scale.z = 1.5;
    speed_marker.color.b = 0;
    speed_marker.color.g = 1;
    speed_marker.color.r = 0;
    speed_marker.color.a = 1;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << object.point.v;
    speed_marker.text = ss.str() + "m/s";
    speed_marker.pose.position.x = object.point.x;
    speed_marker.pose.position.y = object.point.y;
    speed_marker.pose.position.z = object.point.z + 1;
    speed_marker_pub_.publish(speed_marker);
  }
}
}