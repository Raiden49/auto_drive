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
#include <unordered_map>
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
/**
 * @brief 计算欧式距离 
 */
double EuclideanDis(const double& x1, const double& y1, 
                    const double& x2, const double& y2);
/**
 * @brief 寻找路径上的最近点
 */
int SearchMatchIndex(const double& cur_x, const double& cur_y,
                     const std::vector<PathPoint>& way_points,
                     const int& pre_match_index);
/**
 * @brief Get the Projection Point object 计算投影点
 * 
 * @param cur_pose 当前车辆状态
 * @param match_point 路径上匹配点也就是距离最近的点
 * @return PathPoint Cartesian坐标下，计算出来的任意点在Cartesian轨迹上的投影点
 */
PathPoint GetProjectionPoint(const CarState& cur_pose, 
                             const PathPoint& match_point);
/**
 * @brief Get the Projection Point object 计算投影点
 * 
 * @param frenet_point 任意Frenet坐标系下的点
 * @param ref_path Cartesian坐标系下的参考陆军
 * @return PathPoint Cartesian坐标系下，计算出来的任意点在Frenet轨迹上的投影点
 */
PathPoint GetProjectionPoint(const FrenetPoint& frenet_point,
                             const std::vector<PathPoint> ref_path);
/**
 * @brief Cartesian坐标系转换为Frenet坐标系
 * 
 * @param global_point Cartesian坐标点
 * @param projection_point 轨迹上的投影点
 * @return FrenetPoint Frenet坐标系下的点
 */
FrenetPoint Cartesian2Frenet(const CarState& global_point,
                             const std::vector<PathPoint>& ref_path);
/**
 * @brief Frenet坐标系下的点转换到Cartesian坐标系下
 * 
 * @param frenet_point 任意Frenet坐标系下的点
 * @param ref_path Cartesian坐标系下的参考陆军
 */
void Frenet2Cartesian(FrenetPoint& frenet_point, 
                      const std::vector<PathPoint>& ref_path);
class CommonInfo {
 public:
  CommonInfo() = delete;
  /**
   * @brief Construct a new Common Info object 工具类，主要处理回调函数和各种状态值
   * 
   * @param role_name carla中车的名称
   * @param nh 
   */
  CommonInfo(const std::string role_name, ros::NodeHandle& nh) : nh_(nh) {
    lane_width_ = 4.0;
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
    line_width_sub_ = 
        nh_.subscribe("lane/width", 10, &CommonInfo::LaneWdithCallBack, this);
  }
  ~CommonInfo() = default;
 
  /**
   * @brief 定位回调函数，获取到车辆位置后会将flag置为true
   * 
   * @param msg carla发布的odom msg
   */
  void CarlaOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
  /**
   * @brief 巡航速度回调函数，获取到期望速度后更新
   * 
   * @param msg 人为发送的期望速度
   */
  void CruiseSpeedCallBack(const std_msgs::Float32::ConstPtr& msg);
  /**
   * @brief 全局路径回调函数，获取到全局路径后进行更新并将flag置为true
   * 
   * @param msg carla自带的全局规划规划出的轨迹
   */
  void GlobalPathCallBack(const nav_msgs::Path::ConstPtr& msg);
  /**
   * @brief 车辆传感器状态回调函数
   * 
   * @param msg carla发布的车辆状态信息
   */
  void IMUCallBack(const sensor_msgs::Imu::ConstPtr& msg);
  /**
   * @brief 检测到的其他目标回调函数
   * 
   * @param msg carla发布的道路信息
   */
  void DetectedObjectsCallBack(
      const derived_object_msgs::ObjectArray::ConstPtr& msg);
  void LaneWdithCallBack(const std_msgs::Float32::ConstPtr& msg);

 public:
  double lane_width_;
  bool is_odom_received_;                   
  double cruise_speed_;
  CarState cur_pose_;
  ros::NodeHandle nh_;
  std::vector<PathPoint> global_path_;
  std::vector<Obstacle> detected_objects_;

 public:
  ros::Subscriber cur_pose_sub_;
  ros::Subscriber global_path_sub_;
  ros::Subscriber cruise_speed_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber detected_objects_sub_;
  ros::Subscriber line_width_sub_;
};
}

#endif // AUTO_DRIVE_COMMON_HPP_