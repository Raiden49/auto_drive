/*
 * @Author: Raiden49 
 * @Date: 2024-09-19 09:29:02 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 10:08:49
 */
#ifndef REFERENCE_LINE_HPP_
#define REFERENCE_LINE_HPP_

#include "m_common.hpp"
#include "optim/qp_optim.hpp"
#include "optim/b_spline_optim.hpp"

namespace auto_drive {
class ReferenceLine {
 public:
  ReferenceLine(const double& ahead_dist) : ahead_dist_(ahead_dist) {};
  /**
   * @brief Construct a new Reference Line object 参考线处理类
   * 
   * @param ahead_dist 截取的参考线长度
   * @param ref_line_params 参考线各种参数
   */
  ReferenceLine(const double& ahead_dist, 
                std::unordered_map<std::string, double>& ref_line_params) {
    ahead_dist_ = ahead_dist;
    match_index_ = 0;
    ref_weight_smooth_ = ref_line_params["ref_weight_smooth"];
    ref_weight_length_ = ref_line_params["ref_weight_path_length"];
    ref_weight_deviation_ = ref_line_params["ref_weight_ref_deviation"];
    lower_bound_ = ref_line_params["lower_bound"];
    upper_bound_ = ref_line_params["upper_bound"];
  }

  /**
   * @brief 寻找目标长度附近的点的index
   */
  int SearchTargetIndex(const double& cur_x, const double& cur_y,
                        const double& ahead_distance,
                        const std::vector<PathPoint>& way_points);
  /**
   * @brief 截取参考路径为局部
   * 
   * @param cur_pose 当前机器人位置信息
   * @param global_path 全局路径
   * @param pre_match_index 要截取到的匹配点
   * @return std::vector<PathPoint> 截取后的参考线
   */
  std::vector<PathPoint> LocalPathTruncation(const CarState& cur_pose, 
      const std::vector<PathPoint>& global_path, const int& pre_match_index);
  /**
   * @brief 对参考线进行优化，主要使用QP方法
   * 
   * @param local_path 截取后的参考线
   * @return std::vector<PathPoint> 优化后的参考线
   */
  std::vector<PathPoint> LineSmooth(const std::vector<PathPoint>& local_path);
  /**
   * @brief 补充优化后参考线上每个点的信息
   */
  void CalHeading(std::vector<PathPoint>& way_points);

 public:
  int match_index_;                   // 截取的最大index
  double ahead_dist_;                 // 截取的长度
  double ref_weight_smooth_;          // 参考线平滑权重
  double ref_weight_length_;          // 参考线长度权重
  double ref_weight_deviation_;       // 参考线偏离权重
  double lower_bound_;                // QP优化时的下界
  double upper_bound_;                // QP优化时的上界
};
}

#endif // REFERENCE_LINE_HPP_