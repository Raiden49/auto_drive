/*
 * @Author: Raiden49 
 * @Date: 2024-09-20 11:16:10 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-24 15:46:14
 */
#include "collision_detection.hpp"

namespace auto_drive
{
void CollisionDetection::CalCollisionBox(Obstacle& object) {
  std::vector<FrenetPoint> collision_box(8);
  double x = object.point.x, y = object.point.y;
  double yaw = object.point.yaw;
  double x_rad = object.x_rad, y_rad = object.y_rad;

  Eigen::MatrixXd position_matrix(8, 2);
  Eigen::MatrixXd translation_matrix(8, 2);
  Eigen::MatrixXd rotation_matrix(2, 2);
  position_matrix << x, y, x, y, x, y, x, y, x, y, x, y, x, y, x, y;
  translation_matrix << -x_rad, -y_rad, -x_rad, 0, -x_rad, y_rad, 0, y_rad,
                        x_rad, y_rad, x_rad, 0, x_rad, -y_rad, 0, -y_rad;
  rotation_matrix << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);
  position_matrix = translation_matrix * rotation_matrix  + position_matrix;

  for (int i = 0; i < position_matrix.rows(); i++) {
    collision_box[i].x = position_matrix(i, 0);
    collision_box[i].y = position_matrix(i, 1);
    collision_box[i].z = object.point.z;
    collision_box[i].yaw = object.point.yaw;
    collision_box[i].vx = object.point.vx;
    collision_box[i].vy = object.point.vy;
    collision_box[i].v = object.point.v;
  }
  object.collision_box = collision_box;
}
bool CollisionDetection::IsCollision(FrenetPath& path, 
                                     const FrenetPoint& leader_point,
                                     const bool& is_car_followed) {
  for (auto& obstacle : detected_objects_) {
    for (auto& box_point : obstacle.collision_box) {
      for (int i = 0; i < path.size_; i++) {
        double dist = EuclideanDis(path.frenet_points[i].x, path.frenet_points[i].y,
                                   box_point.x, box_point.y);
        if (dist < 3.5 && !(is_car_followed && EuclideanDis(obstacle.point.x, 
                                                            obstacle.point.y, 
                                                            leader_point.x, 
                                                            leader_point.y) < 2.0)) {
          path.cost += dist / 3.0;
        }
        if (dist <= collision_dis_) {
          return true;
        }
      }
    }
  }
  return false; 
}
void CollisionDetection::ObstacleClassification(
    std::vector<Obstacle>& detected_objects) {
  for (auto& obstacle : detected_objects) {
    CalCollisionBox(obstacle);
    if (obstacle.point.v > 0.2) {
      dynamic_obstacle_list_.push_back(obstacle);
    }
    else {
      obstacle.point = CalFrenet(obstacle.point, ref_path_);
      for (auto& box_point : obstacle.collision_box) {
        box_point = CalFrenet(box_point, ref_path_);
      }
      static_obstacle_list_.push_back(obstacle);
    }
  }
}
} // namespace auto_drive
