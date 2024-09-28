/*
 * @Author: Raiden49 
 * @Date: 2024-09-27 10:18:20 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-27 15:02:27
 */
#include "planner/em_planner.hpp"

namespace auto_drive {
namespace planner {
std::vector<std::vector<FrenetPoint>> 
    EMPlanner::DPSampling(const FrenetPoint& initial_frenet_point) {
  std::vector<std::vector<FrenetPoint>> dp_sample_path;
  int rows = this->dp_sample_rows_;
  int cols = this->dp_sample_cols_;
  dp_sample_path.resize(rows);
  std::cout << "row: " << rows << std::endl; 
  for (int i = 0; i < rows; i++) {
    dp_sample_path[i].resize(cols);
    for (int j = 0; j < cols; j++) {
      dp_sample_path[i][j].s = 
          initial_frenet_point.s + (j + 1) * this->dp_sample_s_;
      dp_sample_path[i][j].l = 
          initial_frenet_point.l + ((rows + 1) / 2 - (i + 1)) * this->dp_sample_l_;
      dp_sample_path[i][j].l_ds = 0;
      dp_sample_path[i][j].l_d_ds = 0;

      // std::cout << "dp sample point: " << dp_sample_path[i][j].s << ", " 
      //           << dp_sample_path[i][j].l << std::endl;
    
    }
  }
  return dp_sample_path;
}
double EMPlanner::GetDPPathCost(const FrenetPoint& start_frenet_point,
                                const FrenetPoint& end_frenet_point) {
  auto ls_polynomial = std::make_shared<QuinticPolynomial>(
      start_frenet_point.l, start_frenet_point.l_ds, start_frenet_point.l_d_ds,
      end_frenet_point.l, end_frenet_point.l_ds, end_frenet_point.l_d_ds,
      start_frenet_point.s, end_frenet_point.s);
  
  // 10个点插值计算Cost
  int interpolation_size = 10;
  FrenetPoint interpolation_point;
  double smooth_cost = 0, ref_cost = 0, collision_cost = 0;
  double s_step = (end_frenet_point.s - start_frenet_point.s) / interpolation_size;
  double l_step = (end_frenet_point.s - start_frenet_point.s) / interpolation_size;
  for (int i = 0; i < interpolation_size; i++) {
    interpolation_point.s = start_frenet_point.s + (i + 1) * s_step;
    interpolation_point.l = ls_polynomial->CalPoint(interpolation_point.s);
    interpolation_point.l_ds = 
        ls_polynomial->CalFirstDerivative(interpolation_point.s);
    interpolation_point.l_d_ds = 
        ls_polynomial->CalSecondDerivative(interpolation_point.s);
    interpolation_point.l_d_d_ds = 
        ls_polynomial->CalThirdDerivative(interpolation_point.s);
    smooth_cost += this->dp_cost_dl_ * pow(interpolation_point.l_ds, 2) + 
                   this->dp_cost_ddl_ * pow(interpolation_point.l_d_ds, 2) + 
                   this->dp_cost_dddl_ * pow(interpolation_point.l_d_d_ds, 2);
    if (abs(interpolation_point.l_d_ds) > 0.5 || 
        abs(atan(interpolation_point.l_d_d_ds) > 0.4 * M_PI)) {
      smooth_cost = DBL_MAX;
    }

    ref_cost += this->dp_cost_ref_ * pow(interpolation_point.l, 2);
    
    for (auto& startic_obs : collision_detection_ptr_->static_obstacle_list_) {
      double dis = EuclideanDis(interpolation_point.s, interpolation_point.l,
                                startic_obs.point.s, startic_obs.point.l);
      if (dis < 6.0 && dis > 4.0) {
        collision_cost += 1000;
      }
      else if (dis < 4.0) {
        collision_cost += this->dp_cost_collision_;
      }
    }
  }
  return smooth_cost + ref_cost + collision_cost;
}
std::vector<FrenetPoint> EMPlanner::GetDPPath(const FrenetPoint& initial_frenet_point) {
  
  int rows = this->dp_sample_rows_;
  int cols = this->dp_sample_cols_;
  std::vector<FrenetPoint> dp_path;
  
  auto&& dp_sample_points = DPSampling(initial_frenet_point);
  for (int i = 0; i < rows; i++) {
    dp_sample_points[i][0].dp_cost = GetDPPathCost(initial_frenet_point, 
                                                   dp_sample_points[i][0]);
  }
  for (int j = 1; j < cols; j++) {
    for (int i = 0; i < rows; i++) {
      dp_sample_points[i][j].dp_cost = DBL_MAX;
      for (int k = 0; k < rows; k++) {
        double cost = dp_sample_points[k][j - 1].dp_cost + 
            GetDPPathCost(dp_sample_points[k][j - 1], dp_sample_points[i][j]);
        if (cost < dp_sample_points[i][j].dp_cost) {
          dp_sample_points[i][j].dp_cost = cost;
          dp_sample_points[i][j].dp_pre_row = k;
        }
      }
    }
  }
  // 回溯获得dp最优路径
  dp_path.resize(cols + 1);
  dp_path[0] = initial_frenet_point;
  dp_path[cols] = dp_sample_points[0][cols - 1];
  // 获取最后一列的最优点
  for (int i = 1; i < rows; i++) {
    if (dp_sample_points[i][cols - 1].dp_cost < dp_path[cols].dp_cost) {
      dp_path[cols] = dp_sample_points[i][cols - 1];
    }
  }
  // 回溯获取路径
  for (int j = cols - 1; j > 0; j--) {
    int pre_row = dp_path[j + 1].dp_pre_row;
    dp_path[j] = dp_sample_points[pre_row][j - 1];
  }
  this->dp_sample_points_ = std::move(dp_sample_points);

  // for (auto& dp_point : dp_path) {
  //   std::cout << "dp path point: " << dp_point.s <<  ", " << dp_point.l 
  //             << dp_point.l_ds <<  ", " << dp_point.l_d_ds << std::endl;
  // }

  return dp_path;
}
std::vector<FrenetPoint> EMPlanner::DPPathInterpolation(
    const std::vector<FrenetPoint>& dp_path) {
  std::vector<FrenetPoint> dp_final_path;
  for (int i = 0; i < dp_path.size() - 1; i++) {
    auto polynomial = std::make_shared<QuinticPolynomial>(
        dp_path[i].l, dp_path[i].l_ds, dp_path[i].l_d_ds,
        dp_path[i + 1].l, dp_path[i + 1].l_ds, dp_path[i + 1].l_d_ds,
        dp_path[i].s, dp_path[i + 1].s);
    
    // 五次多项式插点
    for (double s = dp_path[i].s; s <= dp_path[i + 1].s; s += path_ds_) {
      FrenetPoint interpolation_point;
      interpolation_point.s = s;
      interpolation_point.l = polynomial->CalPoint(interpolation_point.s);
      interpolation_point.l_ds = 
          polynomial->CalFirstDerivative(interpolation_point.s);
      interpolation_point.l_d_ds = 
          polynomial->CalSecondDerivative(interpolation_point.s);
      interpolation_point.l_d_d_ds = 
          polynomial->CalThirdDerivative(interpolation_point.s);
      dp_final_path.push_back(interpolation_point);
      
      // std::cout << "interpolation point: " << interpolation_point.s 
      //           << ", " << interpolation_point.l << ", " << interpolation_point.l_ds 
      //           << ", " << interpolation_point.l_d_ds << std::endl;
    }
  }
  return dp_final_path;
}
int EMPlanner::GetObsIndex(const std::vector<FrenetPoint>& dp_final_path, 
                           const double& obs_s) {
  if (dp_final_path.front().s > obs_s) {
    return 0;
  }
  else if (dp_final_path.back().s < obs_s) {
    return dp_final_path.size() - 1;
  }

  int index = 0;
  for (int i = 0; i < dp_final_path.size(); i++) {
    if (dp_final_path[i].s > obs_s) {
      index = i;
      break;
    }
  }
  if (index == 0) {
    return index;
  }
  if (abs(dp_final_path[index].s - obs_s) > abs(dp_final_path[index - 1].s - obs_s)) {
    return index - 1;
  }
  else {
    return index;
  }

  return index;
}
void EMPlanner::GetConvexSpace(const std::vector<FrenetPoint>& dp_final_path,
                              Eigen::VectorXd& l_min, Eigen::VectorXd& l_max) {
  int size = dp_final_path.size();
  l_min = Eigen::VectorXd::Ones(size) * -10;
  l_max = Eigen::VectorXd::Ones(size) * 10;

  for (auto& static_obs : collision_detection_ptr_->static_obstacle_list_) {
    double obs_s_min = static_obs.point.s;
    double obs_s_max = static_obs.point.s;
    double obs_l_min = static_obs.point.l;
    double obs_l_max = static_obs.point.l;
    for (auto& box_point : static_obs.collision_box) {
      obs_s_min = std::min(box_point.s, obs_s_min);
      obs_s_max = std::max(box_point.s, obs_s_max);
      obs_l_min = std::min(box_point.l, obs_l_min);
      obs_l_max = std::max(box_point.l, obs_l_max);
    }

    int start_index = GetObsIndex(dp_final_path, obs_s_min);
    int end_index = GetObsIndex(dp_final_path, obs_s_max);
    int mid_index = GetObsIndex(dp_final_path, static_obs.point.s);
    if (start_index == 0 || end_index == (size - 1)) {
      continue;
    }

    double path_l = dp_final_path[mid_index].l;
    for (int i = start_index; i <= end_index; i++) {
      // 从左边经过
      if (path_l >= static_obs.point.l) {
        l_min(i) = std::max(l_min(i), obs_l_max);
      }
      else {
        l_max(i) = std::min(l_max(i), obs_l_min);
      }
    }
  }
  // l_min = Eigen::VectorXd::Ones(size) * -10;
  // l_max = Eigen::VectorXd::Ones(size) * 10;
}
std::vector<FrenetPoint> EMPlanner::GetQPPath(
    const std::vector<FrenetPoint>& dp_final_path) {
  int n = dp_final_path.size();
  double ds = path_ds_;
  double width = 3.6;     // width of the car

  Eigen::VectorXd l_min, l_max;
  GetConvexSpace(dp_final_path, l_min, l_max);

  std::vector<FrenetPoint> qp_path = dp_final_path;

  Eigen::SparseMatrix<double> H(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_L(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DL(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DDL(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DDDL(n - 1, 3 * n);
  Eigen::SparseMatrix<double> H_CENTRE(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_L_END(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DL_END(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DDL_END(3 * n, 3 * n);

  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * n);
  // Aeq * X = Beq
  Eigen::SparseMatrix<double> Aeq(2 * n - 2, 3 * n);
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(2 * n - 2);

  // lb <= x <= ub
  Eigen::SparseMatrix<double> A_lu(3 * n, 3 * n);
  A_lu.setIdentity();
  Eigen::VectorXd lower_bound = Eigen::VectorXd::Ones(3 * n) * (-DBL_MAX);
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(3 * n) * DBL_MAX;

  // lb_merge <= A_merge * X <= ub_merge
  Eigen::SparseMatrix<double> A_merge(2 * n - 2 + 3 * n, 3 * n);
  Eigen::VectorXd lower_bound_merge(2 * n - 2 + 3 * n);
  Eigen::VectorXd upper_bound_merge(2 * n - 2 + 3 * n);

  for (int i = 0; i < n; i++) {
    H_L.insert(3 * i, 3 * i) = 1;
    H_DL.insert(3 * i + 1, 3 * i + 1) = 1;
    H_DDL.insert(3 * i + 2, 3 * i + 2) = 1;
  }
  H_CENTRE = H_L;
  for (int i = 0; i < n - 1; i++) {
    H_DDDL.insert(i, 3 * i + 2) = 1;
    H_DDDL.insert(i, 3 * i + 5) = -1;
  }
  H_L_END.insert(3 * n - 3, 3 * n - 3) = 0;
  H_DL_END.insert(3 * n - 2, 3 * n - 2) = 0;
  H_DDL_END.insert(3 * n - 1, 3 * n - 1) = 0;
  H = this->qp_cost_l_ * (H_L.transpose() * H_L) + 
      this->qp_cost_dl_ * (H_DL.transpose() * H_DL) + 
      this->qp_cost_ddl_ * (H_DDL.transpose() * H_DDL) + 
      this->qp_cost_dddl_ * (H_DDDL.transpose() * H_DDDL) + 
      this->qp_cost_ref_ * (H_CENTRE.transpose() * H_CENTRE);
  H = 2 * H;

  for (int i = 0; i < n; i++) {
    gradient(3 * i) = -2 * this->qp_cost_ref_ * dp_final_path[i].l;
  }

  // Aep * X = beq
  int row_index_start = 0;
  for (int i = 0; i < n - 1; i++) {
    int row = row_index_start + 2 * i;
    int col = 3 * i;
    A_merge.insert(row, col) = 1;
    A_merge.insert(row, col + 1) = ds;
    A_merge.insert(row, col + 2) = pow(ds, 2) / 3;
    A_merge.insert(row, col + 3) = -1;
    A_merge.insert(row, col + 5) = pow(ds, 2) / 6;
    A_merge.insert(row + 1, col + 1) = 1;
    A_merge.insert(row + 1, col + 2) = ds / 2;
    A_merge.insert(row + 1, col + 4) = -1;
    A_merge.insert(row + 1, col + 5) = ds / 2;
  }
  // lb <= A * x <= ub
  row_index_start = 2 * n - 2;
  for (int i = 0; i < n; i++) {
    int row = row_index_start + 3 * i;
    int col = 3 * i;
    A_merge.insert(row, col) = 1;
    A_merge.insert(row + 1, col + 1) = 1;
    A_merge.insert(row + 2, col + 2) = 1;
  }
  upper_bound(0) = lower_bound(0) = dp_final_path[0].l;
  upper_bound(1) = lower_bound(1) = dp_final_path[0].l_ds;
  upper_bound(2) = lower_bound(2) = dp_final_path[0].l_d_ds;
  for (int i = 1; i < n; i++) {
    lower_bound(3 * i) = l_min(i) + width / 2;
    lower_bound(3 * i + 1) = -2.0;
    lower_bound(3 * i + 2) = -0.1;
    upper_bound(3 * i) = l_max(i) - width / 2;
    upper_bound(3 * i + 1) = 2.0;
    upper_bound(3 * i + 2) = 0.1;
  }

  // merge
  lower_bound_merge.block(0, 0, 2 * n - 2, 1) = beq;
  upper_bound_merge.block(0, 0, 2 * n - 2, 1) = beq;
  lower_bound_merge.block(2 * n - 2, 0, 3 * n, 1) = lower_bound;
  upper_bound_merge.block(2 * n - 2, 0, 3 * n, 1) = upper_bound;

  // osqp solver
  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  solver.settings()->setVerbosity(false);
  solver.data()->setNumberOfVariables(3 * n);
  solver.data()->setNumberOfConstraints(2 * n - 2 + 3 * n);
  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(gradient);
  solver.data()->setLinearConstraintsMatrix(A_merge);
  solver.data()->setLowerBound(lower_bound_merge);
  solver.data()->setUpperBound(upper_bound_merge);

  clock_t start_time = clock();
  try {
      solver.initSolver();
      solver.solve();
  } catch (const std::exception& e) {
      std::cerr << "Error getting problem solved" << e.what() << std::endl;
  }
  clock_t end_time = clock();
  double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
  std::cout << "Get the qp path in " << run_time << " ms" << std::endl;

  Eigen::VectorXd qp_solution(3 * n);
  qp_solution = std::move(solver.getSolution());
  for (int i = 0; i < n; i++) {
    qp_path[i].l = qp_solution(3 * i);
    qp_path[i].l_ds = qp_solution(3 * i + 1);
    qp_path[i].l_d_ds = qp_solution(3 * i + 2);
  }
  return qp_path;
}
int EMPlanner::GetCartesianPath(std::vector<FrenetPoint>& frenet_path, 
                                const std::vector<PathPoint>& ref_path) {
  int size = 0;
  for (int i = 0; i < frenet_path.size(); i++) {
    if (frenet_path[i].s >= ref_path.back().s_) {
      break;
    }
    size++;
    Frenet2Cartesian(frenet_path[i], ref_path);
    frenet_path[i].v = desired_speed_;
  }
  return size;
}
std::vector<FrenetPath> EMPlanner::GetSamplePath(const std::vector<PathPoint>& ref_path) {
  std::vector<FrenetPath> frenet_paths;
  frenet_paths.resize(this->dp_sample_points_.size());
  
  for (int i = 0; i < frenet_paths.size(); i++) {
    GetCartesianPath(this->dp_sample_points_[i], ref_path);
    frenet_paths[i].frenet_points = this->dp_sample_points_[i];
  }
  return frenet_paths;
}
FrenetPath EMPlanner::Planning(const std::vector<PathPoint>& ref_path, 
                               const FrenetPoint& initial_frenet_point) {
  FrenetPath final_path;
  auto&& dp_path = GetDPPath(initial_frenet_point);
  // std::cout << "dp path size: " << dp_path.size() << std::endl;
  auto&& dp_final_path = DPPathInterpolation(dp_path);
  // std::cout << "dp final path size: " << dp_final_path.size() << std::endl;
  auto&& qp_path = GetQPPath(dp_final_path);
  // auto&& qp_path = dp_final_path;
  // std::cout << "qp path size: " << qp_path.size() << std::endl;
  final_path.size_ = GetCartesianPath(qp_path, ref_path);
  final_path.frenet_points = qp_path;
  return final_path;
}
} // namespace planner
} // namespace auto_drive