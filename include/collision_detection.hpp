/*
 * @Author: Raiden49 
 * @Date: 2024-09-18 10:10:21 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-20 16:08:04
 */
#ifndef COLLISION_DETECTION_HPP_
#define COLLISION_DETECTION_HPP_

#include <cfloat>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "m_common.hpp"

namespace auto_drive {

class CollisionDetection {
 public:
  CollisionDetection() = delete;
  CollisionDetection(const std::vector<Obstacle>& detected_objects,
                     const double& collision_distance,
                     const std::vector<PathPoint>& ref_path) : 
                     detected_objects_(detected_objects), 
                     collision_dis_(collision_distance),
                     ref_path_(ref_path) {
    static_obstacle_list_.clear();
    dynamic_obstacle_list_.clear();
    ObstacleClassification(this->detected_objects_);
  }
  void ObstacleClassification(std::vector<Obstacle>& detected_objects);
  void CalCollisionBox(Obstacle& object);
  bool IsCollision(FrenetPath& path, const FrenetPoint& leader_point,
                   const bool& is_car_followed);

 public:
  double collision_dis_;
  std::vector<Obstacle> detected_objects_;
  std::vector<Obstacle> static_obstacle_list_;
  std::vector<Obstacle> dynamic_obstacle_list_;
  std::vector<PathPoint> ref_path_;
};
}

#endif // COLLISTION_DETECTION_HPP_