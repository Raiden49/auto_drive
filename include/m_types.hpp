/*
 * @Author: Raiden49 
 * @Date: 2024-09-14 10:33:43 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-24 15:45:20
 */
#ifndef AUTO_DRIVE_TYPES_HPP_
#define AUTO_DRIVE_TYPES_HPP_

#include <vector>

namespace auto_drive {
struct PathPoint {
 public:
  double x;
  double y;
  double z;
  double yaw;
  double cur;
  double s_;
  PathPoint() = default;
  PathPoint(double x, double y) : x(x), y(y), z(0), yaw(0), cur(0), s_(0) {};
};
struct CarState : public PathPoint {
 public:
  double vx;
  double vy;
  double v;
  double ax;
  double ay;
  double a;
  double t;
};
struct FrenetPoint : public CarState {
 public:
  double s;
  double l;
  double s_d;
  double l_d;
  double s_d_d;
  double l_d_d;
  double s_d_d_d;
  double l_d_d_d;
  double l_ds;
  double l_d_ds;
  double l_d_d_ds;
  double ds;
  double dp_cost;
  int dp_pre_row;
};
struct FrenetPath {
 public:
  int size_ = 0;
  double cost;
  std::vector<FrenetPoint> frenet_points;
  double max_speed;
  double max_acc;
  double max_curvature;
};
struct Obstacle {
 public:
  double x_rad, y_rad;
  FrenetPoint point;
  std::vector<FrenetPoint> collision_box;
};
}

#endif // AUTO_DRIVE_TYPES_HPP_