/*
 * @Author: Raiden49 
 * @Date: 2024-09-19 09:29:02 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 10:08:49
 */
#ifndef REFERENCE_LINE_HPP_
#define REFERENCE_LINE_HPP_

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <OsqpEigen/OsqpEigen.h>

#include "m_common.hpp"
#include "optim/qp_optim.hpp"
#include "optim/b_spline_optim.hpp"

namespace auto_drive {
class ReferenceLine {
 public:
  ReferenceLine(const double& ahead_dist) : ahead_dist_(ahead_dist) {};
  ReferenceLine(const double& ahead_dist, 
                std::unordered_map<std::string, double>& ref_line_params) {
    ahead_dist_ = ahead_dist;
    match_index_ = 0;
    ref_weight_smooth_ = ref_line_params["ref_weight_smooth"];
    ref_weight_length_ = ref_line_params["ref_weight_path_length"];
    ref_weight_deviation_ = ref_line_params["ref_weight_ref_deviation"];
    lower_bound_ = ref_line_params["lower_bound"];
    upper_bound_ = ref_line_params["upper_bound"];
  }

  int SearchTargetIndex(const double& cur_x, const double& cur_y,
                        const double& ahead_distance,
                        const std::vector<PathPoint>& way_points);
  std::vector<PathPoint> LocalPathTruncation(const CarState& cur_pose, 
      const std::vector<PathPoint>& global_path, const int& pre_match_index);
  std::vector<PathPoint> LineSmooth(const std::vector<PathPoint>& local_path);
  void CalHeading(std::vector<PathPoint>& way_points);

 public:
  int match_index_;
  double ahead_dist_;
  double ref_weight_smooth_;
  double ref_weight_length_;
  double ref_weight_deviation_;
  double lower_bound_, upper_bound_;
};
}

#endif // REFERENCE_LINE_HPP_