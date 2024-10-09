/*
 * @Author: Raiden49 
 * @Date: 2024-09-19 11:09:48 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 16:45:50
 */
#include "m_common.hpp"

namespace auto_drive {
double EuclideanDis(const double& x1, const double& y1, 
                    const double& x2, const double& y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}
int SearchMatchIndex(const double& cur_x, const double& cur_y,
                     const std::vector<PathPoint>& way_points,
                     const int& pre_match_index) {
  double dist;
  double min_dist = DBL_MAX;
  int match_index = 0;
  for (int i = pre_match_index; i < way_points.size(); i++) {
    dist = EuclideanDis(way_points[i].x, way_points[i].y, cur_x, cur_y);
    if (dist < min_dist) {
      min_dist = dist;
      match_index = i;
    } 
  }
  return match_index;
}
PathPoint GetProjectionPoint(const CarState& cur_pose, 
                            const PathPoint& match_point) {
  PathPoint projection_point = match_point;

  Eigen::Matrix<double, 2, 1> tor;
  tor << cos(match_point.yaw), sin(match_point.yaw);

  Eigen::Matrix<double, 2, 1> d;
  d << cur_pose.x - match_point.x, cur_pose.y - match_point.y;

  double e_s = tor.transpose() * d;
  projection_point.x = match_point.x + e_s * cos(match_point.yaw);
  projection_point.y = match_point.y + e_s * sin(match_point.yaw);
  projection_point.yaw = match_point.yaw + match_point.cur * e_s;
  return projection_point;
}
PathPoint GetProjectionPoint(const FrenetPoint& frenet_point,
                             const std::vector<PathPoint> ref_path) {
  int closet_index = -1;
  for (int i = 1; i < ref_path.size(); i++) {
    if (ref_path[i].s_ > frenet_point.s) {
      closet_index = i - 1;
      break;
    }
  }
  if (closet_index == -1) {
    ROS_WARN("Can't find the projection point(frenet -> cartesian)");
    return ref_path.back();
  }
  double yaw = (ref_path[closet_index].yaw + ref_path[closet_index + 1].yaw) / 2;
  yaw = ref_path[closet_index].yaw;
  double s_n = ref_path[closet_index].s_;

  double x_r = ref_path[closet_index].x + (frenet_point.s - s_n) * cos(yaw);
  double y_r = ref_path[closet_index].y + (frenet_point.s - s_n) * sin(yaw);

  // std::cout << "yaw: " << yaw << ", " << cos(yaw) << ", " << sin(yaw) << std::endl;

  double yaw_r = ref_path[closet_index].yaw + 
      ref_path[closet_index].cur * (frenet_point.s - s_n);
  double cur_r = (ref_path[closet_index].cur + ref_path[closet_index].cur) / 2;

  PathPoint projection_point;
  projection_point.x = x_r; 
  projection_point.y = y_r;
  projection_point.yaw = yaw_r; 
  projection_point.cur = cur_r;
  // std::cout << "x_r, y_r: " << x_r << "," << y_r << std::endl;
  return projection_point;
}
FrenetPoint Cartesian2Frenet(const CarState& global_point,
                             const std::vector<PathPoint>& ref_path) {

  int frenet_match_index = 
      SearchMatchIndex(global_point.x, global_point.y, ref_path, 0);
  PathPoint projection_point = 
      GetProjectionPoint(global_point, ref_path[frenet_match_index]);

  FrenetPoint frenet_point;
  frenet_point.x = global_point.x; 
  frenet_point.y = global_point.y; 
  frenet_point.z = global_point.z; 
  frenet_point.yaw = global_point.yaw;
  frenet_point.cur = global_point.cur; 
  frenet_point.vx = global_point.vx;
  frenet_point.vy = global_point.vy; 
  frenet_point.v = global_point.v;
  frenet_point.ax = global_point.ax; 
  frenet_point.ay = global_point.ay;
  frenet_point.a = global_point.a; 
  frenet_point.t = global_point.t;

  double delta_theta = global_point.yaw - projection_point.yaw;
  // s
  frenet_point.s = projection_point.s_;
  // l = (x_or - x_r_or) * n_or
  Eigen::Matrix<double, 2, 1> x_or, x_r_or, n_or;
  x_or << global_point.x, global_point.y;
  x_r_or << projection_point.x, projection_point.y;
  n_or << -sin(projection_point.yaw), cos(projection_point.yaw);
  frenet_point.l = (x_or - x_r_or).transpose() * n_or;
  // s_d
  frenet_point.s_d = global_point.v * std::cos(delta_theta) / 
                     (1 - projection_point.cur * frenet_point.l);

  // std::cout << "global_point v: " << global_point.v << std::endl;
  // std::cout << "test: " << global_point.v * std::cos(delta_theta) << std::endl;
  // std::cout << "frenet point sd: " << frenet_point.s_d << std::endl; 

  // ld = v * sin(delta_theta)
  frenet_point.l_d = global_point.v * std::sin(delta_theta);
  // l_ds = l_d / s_d
  if (fabs(frenet_point.s_d) < 1e-6) {
    frenet_point.l_ds = 0;
  }
  else {
    frenet_point.l_ds = frenet_point.l_d / frenet_point.s_d;
  }
  // s_d_d
  Eigen::Matrix<double, 2, 1> t_or, a_or;
  t_or << cos(projection_point.yaw), sin(projection_point.yaw);
  a_or << global_point.ax, global_point.ay;
  frenet_point.s_d_d = (a_or.transpose() * t_or + 
      2 * projection_point.cur * frenet_point.l_ds * pow(frenet_point.s_d, 2)) /
      (1 - projection_point.cur * frenet_point.l);
  // l_d_d = a * sin(delta_theta)
  frenet_point.l_d_d = global_point.a * std::sin(delta_theta);
  // l_d_ds = (l_d_d - l_ds * s_d_d) / s_d^2
  if (fabs(frenet_point.s_d < 1e-6)) {
    frenet_point.l_d_ds = 0;
  }
  else {
    frenet_point.l_d_ds = (frenet_point.l_d_d - 
        frenet_point.l_ds * frenet_point.s_d_d) / pow(frenet_point.s_d, 2); 
  }
  return frenet_point;
}
void Frenet2Cartesian(FrenetPoint& frenet_point, 
                      const std::vector<PathPoint>& ref_path) {
  auto&& projection_point = GetProjectionPoint(frenet_point, ref_path);
  double x_r = projection_point.x, y_r = projection_point.y;
  double theta_r = projection_point.yaw, k_r = projection_point.cur;

  frenet_point.x = x_r - frenet_point.l * sin(theta_r);
  frenet_point.y = y_r + frenet_point.l * cos(theta_r);
  frenet_point.yaw = atan2(frenet_point.l_ds, 1 - k_r * frenet_point.l) + theta_r;
  // just for debug
  // frenet_point.x = frenet_point.s;
  // frenet_point.y = frenet_point.l;
}
void CommonInfo::CarlaOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
  cur_pose_.x = msg->pose.pose.position.x;
  cur_pose_.y = msg->pose.pose.position.y;
  cur_pose_.z = msg->pose.pose.position.z;
  cur_pose_.yaw = tf::getYaw(msg->pose.pose.orientation);
  cur_pose_.vx = msg->twist.twist.linear.x;
  cur_pose_.vy = msg->twist.twist.linear.y;
  cur_pose_.v = sqrt(pow(cur_pose_.vx, 2) + pow(cur_pose_.vy, 2));
  is_odom_received_ = true;
}
void CommonInfo::CruiseSpeedCallBack(const std_msgs::Float32::ConstPtr& msg) {
  cruise_speed_ = msg->data;
  ROS_INFO_STREAM("Desired speed is: " << cruise_speed_);
}
void CommonInfo::GlobalPathCallBack(const nav_msgs::Path::ConstPtr& msg) {
  global_path_.clear();
  for (auto& point : msg->poses) {
    PathPoint global_path_point;
    global_path_point.x = point.pose.position.x;
    global_path_point.y = point.pose.position.y;
    global_path_point.z = point.pose.position.z;
    global_path_point.yaw = tf::getYaw(point.pose.orientation);

    if (global_path_.size() == 0) {
      global_path_.push_back(global_path_point);
    }
    else if (global_path_point.x != global_path_.back().x && 
             global_path_point.y != global_path_.back().y) {
      global_path_.push_back(global_path_point);
    }
  }
  ROS_INFO_STREAM("Global path size: " << global_path_.size());
}
void CommonInfo::IMUCallBack(const sensor_msgs::Imu::ConstPtr& msg) {
  cur_pose_.ax = msg->linear_acceleration.x;
  cur_pose_.ay = msg->linear_acceleration.y;
  cur_pose_.a = sqrt(cur_pose_.ax * cur_pose_.ax + cur_pose_.ay * cur_pose_.ay);
}
void CommonInfo::DetectedObjectsCallBack(
      const derived_object_msgs::ObjectArray::ConstPtr& msg) {
  detected_objects_.clear();
  for (auto& object : msg->objects) {
    Obstacle obs;
    obs.point.x = object.pose.position.x;
    obs.point.y = object.pose.position.y;
    obs.point.z = object.pose.position.z;
    obs.point.vx = object.twist.linear.x;
    obs.point.vy = object.twist.linear.y;
    obs.point.v = sqrt(pow(obs.point.vx, 2) + pow(obs.point.vy, 2));
    obs.point.yaw = tf::getYaw(object.pose.orientation);
    obs.x_rad = object.shape.dimensions[0] / 2.0;
    obs.y_rad = object.shape.dimensions[1] / 2.0;
    
    detected_objects_.push_back(obs);
  }
}
void CommonInfo::LaneWdithCallBack(const std_msgs::Float32::ConstPtr& msg) {
  this->lane_width_ = msg->data;
  ROS_INFO_STREAM("Successfully get the lane width: " << msg->data);
}
} // namespace lattice_planner


