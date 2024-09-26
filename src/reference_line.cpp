/*
 * @Author: Raiden49 
 * @Date: 2024-09-20 10:27:45 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 14:59:21
 */
#include "reference_line.hpp"

namespace auto_drive {
int ReferenceLine::SearchTargetIndex(const double& cur_x, const double& cur_y,
                                     const double& ahead_distance,
                                     const std::vector<PathPoint>& way_points) {
  double dis = 0;
  for (int i = match_index_; i < way_points.size(); i++) {
    dis = EuclideanDis(cur_x, cur_y, way_points[i].x, way_points[i].y);
    if (dis > ahead_distance) {
      return i;
    }
  }
  return way_points.size() - 1;
}
void ReferenceLine::CalHeading(std::vector<PathPoint>& way_points) {
  double x_delta = 0.0, y_delta = 0.0;
  double x_delta_2 = 0.0, y_delta_2 = 0.0;
  for (int i = 0; i < way_points.size(); i++) {
    if (i == 0) {
      x_delta = way_points[i + 1].x - way_points[i].x;
      y_delta = way_points[i + 1].y - way_points[i].y;
      x_delta_2 = way_points[i + 2].x - way_points[i + 1].x;
      y_delta_2 = way_points[i + 2].y - way_points[i + 1].y;
    }
    else if (i == way_points.size() - 1) {
      x_delta = way_points[i].x - way_points[i - 1].x;
      y_delta = way_points[i].y - way_points[i - 1].y;
      x_delta_2 = way_points[i].x - 2 * way_points[i - 1].x + way_points[i - 2].x;
      y_delta_2 = way_points[i].y - 2 * way_points[i - 1].y + way_points[i - 2].y;
    }
    else {
      x_delta = 0.5 * (way_points[i + 1].x - way_points[i - 1].x);
      y_delta = 0.5 * (way_points[i + 1].y - way_points[i - 1].y);
      x_delta_2 = way_points[i + 1].x + way_points[i - 1].x - 2 * way_points[i].x;
      y_delta_2 = way_points[i + 1].y + way_points[i - 1].y - 2 * way_points[i].y;
    }
    way_points[i].yaw = std::atan2(y_delta, x_delta);
    way_points[i].cur = std::abs(y_delta_2 * x_delta - x_delta_2 * y_delta);
    
    way_points[0].s_ = 0;
    for (int i = 1; i < way_points.size(); i++) {
      way_points[i].s_ = EuclideanDis(way_points[i].x, way_points[i].y, 
                                      way_points[i - 1].x, way_points[i - 1].y);
      way_points[i].s_ += way_points[i - 1].s_;
    }
  }
}
std::vector<PathPoint> ReferenceLine::LocalPathTruncation(
    const CarState& cur_pose, const std::vector<PathPoint>& global_path, 
    const int& pre_match_index) {
  this->match_index_ = 
      SearchMatchIndex(cur_pose.x, cur_pose.y, global_path, pre_match_index);
  int target_index = 
      SearchTargetIndex(cur_pose.x, cur_pose.y, ahead_dist_, global_path);
  std::vector<PathPoint> target_path(global_path.begin() + this->match_index_,
                                     global_path.begin() + target_index + 1);
  return target_path;
}
std::vector<PathPoint> ReferenceLine::LineSmooth(
    const std::vector<PathPoint>& local_path) {
  auto optim_ptr = std::make_shared<optimization::QP>(lower_bound_, upper_bound_, 
                                                      ref_weight_smooth_, 
                                                      ref_weight_length_,
                                                      ref_weight_deviation_, 
                                                      local_path);
  auto smoothed_path = optim_ptr->Process();
  CalHeading(smoothed_path);
  return smoothed_path;
}
}