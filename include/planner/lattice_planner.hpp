/*
 * @Author: Raiden49 
 * @Date: 2024-09-23 17:23:33 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 15:32:18
 */
#ifndef LATTICE_PLANNER_HPP_
#define LATTICE_PLANNER_HPP_

#include <unordered_map>
#include <queue>

#include "m_common.hpp"
#include "collision_detection.hpp"
#include "quintic_polynomial.hpp"

namespace auto_drive {
namespace planner {
class Cmp {
 public:
  bool operator()(const FrenetPath& a, const FrenetPath& b) {
    return a.cost > b.cost;
  }
};
class LatticePlanner {
 public:
  LatticePlanner() = delete;
  LatticePlanner(const double& cruise_speed,
                 std::unordered_map<std::string, double>& lattice_params,
                 std::shared_ptr<CollisionDetection> collision_detection_ptr) {
    this->sample_max_time_ = lattice_params["sample_max_time"];
    this->sample_min_time_ = lattice_params["sample_min_time"];
    this->sample_time_step_ = lattice_params["sample_time_step"];
    this->sample_lat_width_ = lattice_params["sample_lat_width"];
    this->sample_width_length_ = lattice_params["sample_width_length"];
    this->weight_st_object_ = lattice_params["weight_st_object"];
    this->weight_st_jerk_ = lattice_params["weight_st_jerk"];
    this->weight_lt_offset_ = lattice_params["weight_lt_offset"];
    this->weight_lt_acc_ = lattice_params["weight_lt_acc"];
    this->cruise_speed_ = cruise_speed;
    this->collision_detection_ptr_ = collision_detection_ptr;
  }

  void GetStValues(FrenetPoint& frenet_point, 
                   const std::shared_ptr<QuinticPolynomial> st_polynomial);
  void GetLtValues(FrenetPoint& frenet_point, 
                   const std::shared_ptr<QuinticPolynomial> lt_polynomial);
  double GetStObjectCost(const FrenetPath& frenet_path, const double& target_speed);
  double GetStJerkCost(const FrenetPath& frenet_path);
  double GetLtOffsetCost(const FrenetPath& frenet_path, 
                         const FrenetPoint& initial_frenet_point);
  double GetLtAccCost(const FrenetPath& frenet_path);
  std::vector<FrenetPath> SamplingFollowingFrenetPaths(
      const FrenetPoint& initial_frenet_point, 
      const FrenetPoint& leader_frenet_point);
  std::vector<FrenetPath> SamplingCruisingFrenetPaths(
      const FrenetPoint& initial_frenet_point);
  void GetCartesianPaths(std::vector<FrenetPath>& frenet_paths, 
                         const std::vector<PathPoint>& ref_path);
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, Cmp> GetValidPaths(
      std::vector<FrenetPath>& frenet_paths, const FrenetPoint& leader_frenet_point, 
      const bool& is_car_followed);
  std::vector<FrenetPath> GetCandidatePaths(
      const std::vector<PathPoint>& ref_path, const FrenetPoint& initial_frenet_point,
      const FrenetPoint& leader_frenet_point, const bool& is_car_followed);
 
 public:
   const double MAX_SPEED = 50.0 / 3.6;
   const double MAX_ACCEL = 8.0;
   const double MAX_CURVATURE = 100.0;
  // sample params
  double sample_max_time_, sample_min_time_, sample_time_step_;
  double sample_lat_width_, sample_width_length_;
  // weight params
  double weight_st_object_, weight_st_jerk_;
  double weight_lt_offset_, weight_lt_acc_;
  
  double desired_speed_, cruise_speed_;
  FrenetPath best_path_, pre_best_path_;
  std::shared_ptr<CollisionDetection> collision_detection_ptr_;
};
}
}

#endif // LATTICE_PLANNER_HPP_