/*
 * @Author: Raiden49 
 * @Date: 2024-09-27 09:56:48 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-27 14:57:16
 */
#ifndef EM_PLANNER_HPP_
#define EM_PLANNER_HPP_

#include "collision_detection.hpp"
#include "quintic_polynomial.hpp"

namespace auto_drive {
namespace planner {
class EMPlanner {
 public:
  EMPlanner() = delete;
  /**
   * @brief Construct a new EMPlanner object
   * 
   * @param dp_path_params 动态规划参数
   * @param qp_path_params 二次规划优化参数
   * @param collision_detection_ptr 碰撞检测模块
   */
  EMPlanner(std::unordered_map<std::string, double>& dp_path_params,
            std::unordered_map<std::string, double>& qp_path_params,
            std::shared_ptr<CollisionDetection> collision_detection_ptr) : 
            collision_detection_ptr_(collision_detection_ptr) {
    // DP params
    this->dp_sample_l_ = dp_path_params["dp_sample_l"];
    this->dp_sample_s_ = dp_path_params["dp_sample_s"];
    this->dp_sample_rows_ = dp_path_params["dp_sample_rows"];
    this->dp_sample_cols_ = dp_path_params["dp_sample_cols"];
    this->dp_cost_collision_ = dp_path_params["dp_cost_collision"];
    this->dp_cost_dl_ = dp_path_params["dp_cost_dl"];
    this->dp_cost_ddl_ = dp_path_params["dp_cost_ddl"];
    this->dp_cost_dddl_ = dp_path_params["dp_cost_dddl"];
    this->dp_cost_ref_ = dp_path_params["dp_cost_ref"];
    // QP params
    this->qp_cost_l_ = qp_path_params["qp_cost_l"];
    this->qp_cost_dl_ = qp_path_params["qp_cost_dl"];
    this->qp_cost_ddl_ = qp_path_params["qp_cost_ddl"];
    this->qp_cost_dddl_ = qp_path_params["qp_cost_dddl"];
    this->qp_cost_ref_ = qp_path_params["qp_cost_ref"];
    this->qp_cost_end_l_ = qp_path_params["qp_cost_end_l"];
    this->qp_cost_end_dl_ = qp_path_params["qp_cost_end_dl"];
    this->qp_cost_end_ddl_ = qp_path_params["qp_cost_end_ddl"];
  }
  ~EMPlanner() = default;

  // DP path
  /**
   * @brief 采样所有曲线
   * 
   * @param initial_frenet_point Frenet坐标系下的初始点
   * @return std::vector<std::vector<FrenetPoint>> 
   */
  std::vector<std::vector<FrenetPoint>> DPSampling(
      const FrenetPoint& initial_frenet_point);
  /**
   * @brief 获取动态规划选择出路径的cost
   * 
   * @param start_frenet_point Frenet坐标系下的起点
   * @param end_frenet_point Frenet坐标系下的终点
   * @return double cost
   */
  double GetDPPathCost(const FrenetPoint& start_frenet_point,
                       const FrenetPoint& end_frenet_point);
  /**
   * @brief 动态规划获取最优路径
   * 
   * @param initial_frenet_point Frenet坐标系下的初始点
   * @return std::vector<FrenetPoint> 动态规划选择出的最优路径
   */
  std::vector<FrenetPoint> GetDPPath(const FrenetPoint& initial_frenet_point);
  /**
   * @brief DP选择出的路径进行插值
   * 
   * @param dp_path 插值前的dp路径
   * @return std::vector<FrenetPoint> 插值后的dp路径 
   */
  std::vector<FrenetPoint> DPPathInterpolation(
      const std::vector<FrenetPoint>& dp_path);
  // QP path
  /**
   * @brief 二次规划优化dp最优路径
   * 
   * @param dp_final_path dp选择出的最优路径
   * @return std::vector<FrenetPoint> 二次规划后的路径
   */
  std::vector<FrenetPoint> GetQPPath(const std::vector<FrenetPoint>& dp_final_path);
  /**
   * @brief 获取解空间，即变量可行的上下界
   * 
   * @param dp_final_path dp选择出的最优路径
   * @param l_min 变量下界
   * @param l_max 变量上界
   */
  void GetConvexSpace(const std::vector<FrenetPoint>& dp_final_path,
                      Eigen::VectorXd& l_min, Eigen::VectorXd& l_max);
  /**
   * @brief 获取障碍物对应的路径点的坐标
   * 
   * @param dp_final_path dp选择出的最优路径
   * @param obs_s 障碍物在sl坐标系下的s
   * @return int 障碍物对应的路径点的坐标
   */
  int GetObsIndex(const std::vector<FrenetPoint>& dp_final_path, 
                  const double& obs_s);
  /**
   * @brief 获取最后Cartesian坐标系下的路径
   * 
   * @param frenet_path Frenet坐标系下的路径
   * @param ref_path 参考路径
   * @return int 
   */
  int GetCartesianPath(std::vector<FrenetPoint>& frenet_path, 
                       const std::vector<PathPoint>& ref_path);
  /**
   * @brief 主要是为了可视化，读取采样的路径
   */
  std::vector<FrenetPath> GetSamplePath(const std::vector<PathPoint>& ref_path);
  FrenetPath Planning(const std::vector<PathPoint>& ref_path, 
                      const FrenetPoint& initial_frenet_point);

 public:
  double path_ds_ = 0.1;        // 路径间隔
  double desired_speed_ = 8.0;  // 期望速度
  double lane_left_l_ = 0;      // 道路左边界
  double lane_right_l_ = 0;     // 道路右边界
  std::vector<std::vector<FrenetPoint>> dp_sample_points_;
  std::shared_ptr<CollisionDetection> collision_detection_ptr_;
 
 private:
  int dp_sample_rows_;
  int dp_sample_cols_;
  double dp_sample_l_;
  double dp_sample_s_;
  double dp_cost_collision_;
  double dp_cost_dl_;         // 切向速度
  double dp_cost_ddl_;        // 曲率
  double dp_cost_dddl_;       // 曲率变化率
  double dp_cost_ref_;

 private:
  double qp_cost_l_;
  double qp_cost_dl_;
  double qp_cost_ddl_;
  double qp_cost_dddl_;
  double qp_cost_ref_;
  double qp_cost_end_l_;
  double qp_cost_end_dl_;
  double qp_cost_end_ddl_;
};
}
}

#endif // EM_PLANNER_HPP_