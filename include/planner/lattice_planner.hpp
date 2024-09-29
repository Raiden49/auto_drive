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
#include "quartic_polynomial.hpp"

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
  /**
   * @brief Construct a new Lattice Planner object
   * 
   * @param cruise_speed 巡航速度
   * @param lattice_params 规划器的参数
   * @param collision_detection_ptr 碰撞类
   */
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

  /**
   * @brief Get the St Values object
   * 
   * @param frenet_point 当前Frenet坐标系下的点
   * @param st_polynomial 纵向五次多项式
   */
  void GetStValues(FrenetPoint& frenet_point, 
                   const std::shared_ptr<QuarticPolynomial> st_polynomial);
  
  /**
   * @brief Get the St Values object
   * 
   * @param frenet_point 当前Frenet坐标系下的点
   * @param st_polynomial 纵向四次多项式
   */
  void GetStValues(FrenetPoint& frenet_point, 
                   const std::shared_ptr<QuinticPolynomial> st_polynomial);
  /**
   * @brief Get the Lt Values object
   * 
   * @param frenet_point 当前Frenet坐标系下的点
   * @param lt_polynomial 横向五次多项式
   */
  void GetLtValues(FrenetPoint& frenet_point, 
                   const std::shared_ptr<QuinticPolynomial> lt_polynomial);
  /**
   * @brief Get the St Object Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @param target_speed 目标速度
   * @return double 纵向目标代价
   */
  double GetStObjectCost(const FrenetPath& frenet_path, const double& target_speed);
  /**
   * @brief Get the St Jerk Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @return double 纵向舒适代价
   */
  double GetStJerkCost(const FrenetPath& frenet_path);
  /**
   * @brief Get the Lt Offset Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @return double 横向偏离代价
   */
  double GetLtOffsetCost(const FrenetPath& frenet_path, 
                         const FrenetPoint& initial_frenet_point);
  /**
   * @brief Get the Lt Acc Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @return double 横向舒适代价
   */
  double GetLtAccCost(const FrenetPath& frenet_path);
  /**
   * @brief 跟车时的路径生成方式
   * 
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @param leader_frenet_point 前车在Frenet坐标系下的坐标
   * @return std::vector<FrenetPath> 待进一步选择的轨迹族，Frenet坐标系下
   */
  std::vector<FrenetPath> SamplingFollowingFrenetPaths(
      const FrenetPoint& initial_frenet_point, 
      const FrenetPoint& leader_frenet_point);
  /**
   * @brief 无跟车时的路径生成方式
   * 
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @return std::vector<FrenetPath> 待进一步选择的轨迹族，Frenet坐标系下
   */
  std::vector<FrenetPath> SamplingCruisingFrenetPaths(
      const FrenetPoint& initial_frenet_point);
  /**
   * @brief Get the Cartesian Paths object 将Frenet坐标系下的路径转换为Cartesian下
   * 
   * @param frenet_paths 待转换的Frenet下路经
   * @param ref_path 参考路径（Cartesian下）
   */
  void GetCartesianPaths(std::vector<FrenetPath>& frenet_paths, 
                         const std::vector<PathPoint>& ref_path);
  /**
   * @brief Get the Valid Paths object 过滤不合格的路径（比如有障碍物，速度加速度曲率不合格）
   * 
   * @param frenet_paths 待选择的Frenet下路经
   * @param leader_frenet_point 前车在Frenet坐标系下的坐标
   * @param is_car_followed 是否正在跟车的flag
   * @return std::priority_queue<FrenetPath, std::vector<FrenetPath>, Cmp> 
   * 最小堆，第一个即为cost最小的路径
   */
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, Cmp> GetValidPaths(
      std::vector<FrenetPath>& frenet_paths, const FrenetPoint& leader_frenet_point, 
      const bool& is_car_followed);
  /**
   * @brief Get the Candidate Paths object 计算局部路径族
   * 
   * @param ref_path 参考路径（Cartesian下）
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @param leader_frenet_point 前车在Frenet坐标系下的坐标
   * @param is_car_followed 是否正在跟车的flag
   * @return std::vector<FrenetPath> 未经选择的Frenet坐标系下的所有生成轨迹
   */
  std::vector<FrenetPath> GetCandidatePaths(
      const std::vector<PathPoint>& ref_path, const FrenetPoint& initial_frenet_point,
      const FrenetPoint& leader_frenet_point, const bool& is_car_followed);
 
 public:
   const double MAX_SPEED = 50.0 / 3.6;
   const double MAX_ACCEL = 8.0;
   const double MAX_CURVATURE = 100.0;
  // sample params
  double sample_max_time_;        // 最大纵向采样时间
  double sample_min_time_;        // 最小纵向采样时间
  double sample_time_step_;       // 采样间隔
  double sample_lat_width_;       // 横向采样距离
  double sample_width_length_;    // 横向采样间隔
  // weight params
  double weight_st_object_;       // 纵向目标代价
  double weight_st_jerk_;         // 纵向舒适代价
  double weight_lt_offset_;       // 横向偏离代价
  double weight_lt_acc_;          // 横向舒适代价
  
  double desired_speed_;          // 期望速度
  double cruise_speed_;           // 巡航速度
  FrenetPath best_path_;          // 当前周期的最优轨迹
  FrenetPath pre_best_path_;      // 上一周期的最优轨迹
  // 碰撞检测模块
  std::shared_ptr<CollisionDetection> collision_detection_ptr_;
};
}
}

#endif // LATTICE_PLANNER_HPP_