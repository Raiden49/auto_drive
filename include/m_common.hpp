/*
 * @Author: Raiden49 
 * @Date: 2024-09-19 10:55:52 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 11:35:02
 */
#ifndef AUTO_DRIVE_COMMON_HPP_
#define AUTO_DRIVE_COMMON_HPP_

#include <vector>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include "m_types.hpp"

namespace auto_drive {
double EuclideanDis(const double& x1, const double& y1, 
                    const double& x2, const double& y2);
int SearchMatchIndex(const double& cur_x, const double& cur_y,
                     const std::vector<PathPoint>& way_points,
                     const int& pre_match_index);
PathPoint GetProjectionPoint(const CarState& cur_pose, 
                             const PathPoint& match_point);
PathPoint GetProjectionPoint(const FrenetPoint& frenet_point,
                             const std::vector<PathPoint> ref_path);
FrenetPoint Cartesian2Frenet(const CarState& global_point,
                             const PathPoint& projection_point);
FrenetPoint CalFrenet(const CarState& global_point, 
                      const std::vector<PathPoint>& ref_path);
void Frenet2Cartesian(FrenetPoint& frenet_point, 
                      const std::vector<PathPoint>& ref_path);
class CommonInfo {
 public:
  CommonInfo() = delete;
  CommonInfo(const std::string role_name, ros::NodeHandle& nh) : nh_(nh) {
    cruise_speed_ = 5.0;
    is_odom_received_ = false;
    detected_objects_.clear();

    // setup subscriber
    cur_pose_sub_ = nh_.subscribe("/carla/" + role_name + "/odometry", 
        10, &CommonInfo::CarlaOdomCallBack, this);
    global_path_sub_ = nh_.subscribe("/carla/" + role_name + "/waypoints", 
        10, &CommonInfo::GlobalPathCallBack, this);
    cruise_speed_sub_ = nh_.subscribe("/cruise_speed", 
        10, &CommonInfo::CruiseSpeedCallBack, this);
    imu_sub_ = nh_.subscribe("/carla/" + role_name + "/imu", 
        10, &CommonInfo::IMUCallBack, this);
    detected_objects_sub_ = nh_.subscribe("/carla/" + role_name + "/objects", 
        10, &CommonInfo::DetectedObjectsCallBack, this);
  }
  ~CommonInfo() = default;
 
  void CarlaOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
  void CruiseSpeedCallBack(const std_msgs::Float32::ConstPtr& msg);
  void GlobalPathCallBack(const nav_msgs::Path::ConstPtr& msg);
  void IMUCallBack(const sensor_msgs::Imu::ConstPtr& msg);
  void DetectedObjectsCallBack(
      const derived_object_msgs::ObjectArray::ConstPtr& msg);

 public:
  ros::NodeHandle nh_;
  bool is_odom_received_;
  double cruise_speed_;
  CarState cur_pose_;
  std::vector<PathPoint> global_path_;
  std::vector<Obstacle> detected_objects_;

 public:
  ros::Subscriber cur_pose_sub_;
  ros::Subscriber global_path_sub_;
  ros::Subscriber cruise_speed_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber detected_objects_sub_;
};
}

#endif // AUTO_DRIVE_COMMON_HPP_