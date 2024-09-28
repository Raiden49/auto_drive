/*
 * @Author: Raiden49 
 * @Date: 2024-09-24 15:21:25 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 16:04:42
 */
#include "planner/lattice_planner.hpp"

namespace auto_drive{
namespace planner
{
void LatticePlanner::GetStValues(FrenetPoint& frenet_point, 
    const std::shared_ptr<QuinticPolynomial> st_polynomial) {
  frenet_point.s = st_polynomial->CalPoint(frenet_point.t);
  frenet_point.s_d = st_polynomial->CalFirstDerivative(frenet_point.t);
  frenet_point.s_d_d = st_polynomial->CalSecondDerivative(frenet_point.t);
  frenet_point.s_d_d_d = st_polynomial->CalThirdDerivative(frenet_point.t);
  frenet_point.v = desired_speed_;
}
void LatticePlanner::GetLtValues(FrenetPoint& frenet_point, 
    const std::shared_ptr<QuinticPolynomial> lt_polynomial) {
  frenet_point.l = lt_polynomial->CalPoint(frenet_point.t);
  frenet_point.l_d = lt_polynomial->CalFirstDerivative(frenet_point.t);
  frenet_point.l_d_d = lt_polynomial->CalSecondDerivative(frenet_point.t);
  frenet_point.l_d_d_d = lt_polynomial->CalThirdDerivative(frenet_point.t);
}
double LatticePlanner::GetStObjectCost(const FrenetPath& frenet_path, 
                                       const double& target_speed) {
  double object_cost = 0.0, speed_cost = 0.0;
  double time_square_sum = 0.1, dist_cost = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    speed_cost += pow(frenet_point.t, 2) * fabs(target_speed - frenet_point.s_d);
    time_square_sum += pow(frenet_point.t, 2);
  }
  speed_cost = speed_cost / time_square_sum;
  dist_cost = 1.0 / (1.0 + frenet_path.frenet_points.back().s);
  object_cost = (speed_cost + 10 * dist_cost) / 11;

  return object_cost;
}
double LatticePlanner::GetStJerkCost(const FrenetPath& frenet_path) {
  double st_jerk_cost = 0.0, jerk_square_sum = 0.0, jerk_abs_sum = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    jerk_square_sum += pow(frenet_point.s_d_d_d / 4.0, 2);
    jerk_abs_sum += fabs(frenet_point.s_d_d_d / 4.0);
  }
  st_jerk_cost = jerk_square_sum / jerk_abs_sum;
  
  return st_jerk_cost;
}
double LatticePlanner::GetLtOffsetCost(const FrenetPath& frenet_path, 
                                       const FrenetPoint& initial_frenet_point) {
  double lt_offset_cost = 0.0, offset_square_sum = 0.0, offset_abs_sum = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    // 反向则放大代价值
    if (frenet_point.l * initial_frenet_point.l < 0.0) {
      offset_square_sum += pow(frenet_point.l / 3.5, 2) * 5;
      offset_abs_sum += fabs(frenet_point.l / 3.5) * 5;
    }
    else {
      offset_square_sum += pow(frenet_point.l / 3.5, 2);
      offset_abs_sum += fabs(frenet_point.l / 3.5);
    }
  }
  lt_offset_cost = offset_square_sum / offset_abs_sum;
  
  return lt_offset_cost;
}
double LatticePlanner::GetLtAccCost(const FrenetPath& frenet_path) {
  double max_acc = 0.0, lt_acc_cost = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    if (fabs(max_acc) < fabs(frenet_point.l_d_d)) {
      max_acc = frenet_point.l_d_d;
    }
  }
  lt_acc_cost = fabs(max_acc);

  return lt_acc_cost;
}
std::vector<FrenetPath> LatticePlanner::SamplingFollowingFrenetPaths(
    const FrenetPoint& initial_frenet_point, 
    const FrenetPoint& leader_frenet_point) {
  std::vector<FrenetPath> frenet_paths;
  for (double t_i = sample_min_time_; t_i <= sample_max_time_;) {
    t_i += sample_time_step_;
    auto st_polynomial = std::make_shared<QuinticPolynomial>(
        initial_frenet_point.s, initial_frenet_point.s_d, 0.0, 
        leader_frenet_point.s - 8.0, 
        std::min(cruise_speed_, leader_frenet_point.v), 0.0, 0.0, t_i);
    
    for (double l_i = -0.5 * sample_lat_width_; l_i <= 0.5 * sample_lat_width_;) {
      l_i += sample_width_length_;
      auto lt_polynomial = std::make_shared<QuinticPolynomial>(
          initial_frenet_point.l, initial_frenet_point.l_d, 
          initial_frenet_point.l_d_d, l_i, 0.0, 0.0, 0.0, t_i);
      
      FrenetPath frenet_path;
      frenet_path.max_speed = std::numeric_limits<double>::min();
      frenet_path.max_acc = std::numeric_limits<double>::min();
      for (double t = 0; t < t_i; t += 0.02) {
        FrenetPoint frenet_point;
        frenet_point.t = t;
        GetStValues(frenet_point, st_polynomial);
        GetLtValues(frenet_point, lt_polynomial);
        frenet_path.frenet_points.push_back(frenet_point);
        
        frenet_path.max_speed = frenet_point.s_d > frenet_path.max_speed ? 
            frenet_point.s_d : frenet_path.max_speed;
        frenet_path.max_acc = frenet_point.s_d_d > frenet_path.max_acc ? 
            frenet_point.s_d_d : frenet_path.max_acc;
      }
      double st_object_cost = GetStObjectCost(frenet_path, cruise_speed_);
      double st_jerk_cost = GetStJerkCost(frenet_path);
      double lt_offset_cost = GetLtOffsetCost(frenet_path, initial_frenet_point);
      double lt_acc_cost = GetLtAccCost(frenet_path);
      frenet_path.cost = weight_st_object_ * st_object_cost +
                         weight_st_jerk_ * st_jerk_cost + 
                         weight_lt_offset_ * lt_offset_cost + 
                         weight_lt_acc_ * lt_acc_cost;
      frenet_paths.push_back(frenet_path);
    }
  }
  return frenet_paths;
}
std::vector<FrenetPath> LatticePlanner::SamplingCruisingFrenetPaths(
    const FrenetPoint& initial_frenet_point) {
  std::vector<FrenetPath> frenet_paths;
  for (double t_i = sample_max_time_; t_i <= sample_max_time_;) {
    t_i += sample_time_step_;
    auto st_polynomial = std::make_shared<QuinticPolynomial>(
        initial_frenet_point.s, initial_frenet_point.s_d, 0.0, 
        initial_frenet_point.s + 20.0, cruise_speed_, 0.0, 0.0, t_i);

    for (double l_i = -1 * sample_lat_width_; l_i <= sample_lat_width_;) {
      l_i += sample_width_length_;
      auto lt_polynomial = std::make_shared<QuinticPolynomial>(
          initial_frenet_point.l, initial_frenet_point.l_d, 
          initial_frenet_point.l_d_d, l_i, 0.0, 0.0, 0.0, t_i);
      FrenetPath frenet_path;
      frenet_path.max_speed = std::numeric_limits<double>::min();
      frenet_path.max_acc = std::numeric_limits<double>::min();
      
      for (double t = 0; t <= t_i; t += 0.02) {
        FrenetPoint frenet_point;
        frenet_point.t = t;
        GetStValues(frenet_point, st_polynomial);
        GetLtValues(frenet_point, lt_polynomial);
        frenet_path.frenet_points.push_back(frenet_point);
        
        frenet_path.max_speed = frenet_point.s_d > frenet_path.max_speed ? 
            frenet_point.s_d : frenet_path.max_speed;
        frenet_path.max_acc = frenet_point.s_d_d > frenet_path.max_acc ? 
            frenet_point.s_d_d : frenet_path.max_acc;
      }
      double st_object_cost = GetStObjectCost(frenet_path, cruise_speed_);
      double st_jerk_cost = GetStJerkCost(frenet_path);
      double lt_offset_cost = GetLtOffsetCost(frenet_path, initial_frenet_point);
      double lt_acc_cost = GetLtAccCost(frenet_path);
      frenet_path.cost = weight_st_object_ * st_object_cost +
                         weight_st_jerk_ * st_jerk_cost + 
                         weight_lt_offset_ * lt_offset_cost + 
                         weight_lt_acc_ * lt_acc_cost;
      frenet_paths.push_back(frenet_path);
    }
  }
  return frenet_paths;
}
void LatticePlanner::GetCartesianPaths(std::vector<FrenetPath>& frenet_paths, 
                                       const std::vector<PathPoint>& ref_path) {
  for (auto& frenet_path : frenet_paths) {
    frenet_path.size_ = 0;
    for (int i = 0; i < frenet_path.frenet_points.size(); i++) {
      if (frenet_path.frenet_points[i].s >= ref_path.back().s_) {
        break;
      }
      frenet_path.size_++;
    }
    for (int i = 0; i< frenet_path.size_; i++) {
      Frenet2Cartesian(frenet_path.frenet_points[i], ref_path);
      frenet_path.frenet_points[i].v = desired_speed_;
    }
  }
}
std::priority_queue<FrenetPath, std::vector<FrenetPath>, Cmp> 
    LatticePlanner::GetValidPaths(std::vector<FrenetPath>& frenet_paths, 
                                  const FrenetPoint& leader_frenet_point, 
                                  const bool& is_car_followed) {
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, Cmp> valid_paths;
  for (auto& frenet_path : frenet_paths) {
    if (frenet_path.max_speed < MAX_SPEED && 
        frenet_path.max_acc < MAX_ACCEL && 
        frenet_path.max_curvature < MAX_CURVATURE && 
        !collision_detection_ptr_->IsCollision(frenet_path, 
                                               leader_frenet_point,
                                               is_car_followed)) {
      valid_paths.push(frenet_path);
    }
  }
  return valid_paths;
}
std::vector<FrenetPath> LatticePlanner::GetCandidatePaths(
    const std::vector<PathPoint>& ref_path, const FrenetPoint& initial_frenet_point,
    const FrenetPoint& leader_frenet_point, const bool& is_car_followed) {
  std::vector<FrenetPath> frenet_paths;
  if (is_car_followed) {
    desired_speed_ = std::min(cruise_speed_, leader_frenet_point.v);
    frenet_paths = std::move(SamplingFollowingFrenetPaths(initial_frenet_point, 
                                                          leader_frenet_point));
  } 
  else {
    desired_speed_ = cruise_speed_;
    frenet_paths = std::move(SamplingCruisingFrenetPaths(initial_frenet_point));
  }
  GetCartesianPaths(frenet_paths, ref_path);
  auto&& valid_paths = 
      GetValidPaths(frenet_paths, leader_frenet_point, is_car_followed);
  
  std::vector<FrenetPath> planning_paths;
  while (!valid_paths.empty()) {
    planning_paths.push_back(valid_paths.top());
    valid_paths.pop();
  }
  return planning_paths;
}
} // namespace planner
} // auto_drive