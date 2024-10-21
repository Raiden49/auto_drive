/*
 * @Author: Raiden49 
 * @Date: 2024-10-12 14:56:58 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:31:28
 */
#include "planner/cilqr.hpp"

namespace auto_drive {
namespace cilqr_planner {
bool CiLQR::SolverInit(casadi::SX& state, casadi::SX& control) {
  try {
    // 运动学模型构建
    // state size: n x 1; control size: m x 1
    auto f = vehicle_model_ptr_->VehicleModelConstruct(state, control);
    auto f_x = jacobian(f, state);          // n x n 
    auto f_u = jacobian(f, control);        // n x m

    f_ = casadi::Function("f", {state, control}, {f});      
    f_x_ = casadi::Function("f_x", {state, control}, {f_x});
    f_u_ = casadi::Function("f_u", {state, control}, {f_u});

    // Cost函数模型构建
    auto l = constraints_ptr_->CostConstruct(state, control);
    auto l_x = jacobian(l, state).T();      // 1 x n -> n x 1
    auto l_xx = hessian(l, state);          // n x n
    auto l_u = jacobian(l, control).T();    // 1 x m -> m x 1
    auto l_uu = hessian(l, control);        // m x m
    auto l_ux = jacobian(l_u, state);       // m x n

    l_ = casadi::Function("l", {state, control}, {l});
    l_x_ = casadi::Function("l_x", {state, control}, {l_x});
    l_xx_ = casadi::Function("l_xx", {state, control}, {l_xx});
    l_u_ = casadi::Function("l_u", {state, control}, {l_u});
    l_uu_ = casadi::Function("l_uu", {state, control}, {l_uu});
    l_ux_ = casadi::Function("l_ux", {state, control}, {l_ux});


    all_possible_trajs_.clear();
    auto [state_debug, control_debug, obs_debug] = constraints_ptr_->CostDebug(state, 
                                                                               control);
    l_debug_ = casadi::Function("l_debug", {state, control}, {state_debug, 
                                                              control_debug, 
                                                              obs_debug});
  }
  catch (const std::exception& e) {
    std::cerr << "Model constructed failed!!!" << e.what() << std::endl;
  }

  return true;
}
std::tuple<casadi::DM, casadi::DM> CiLQR::Loop(const casadi::DM& x0, 
                                               const casadi::DM& us_init) {
  casadi::SX state = casadi::SX::sym("x", params_["num_state"]);
  casadi::SX control = casadi::SX::sym("u", params_["num_control"]);
  if (!SolverInit(state, control)) {
    std::cerr << "Model has not been constructed!!!" << std::endl;
  }    

  casadi::DM xs = RollOut(x0, us_init);
  casadi::DM us = us_init;
  
  double j_old = __DBL_MAX__;
  double regu = 1;

  for (int it = 0; it < params_["max_iters"]; it++) {
    auto [ks, Ks, current_cost] = BackwardPass(xs, us, regu);
    if (abs(current_cost) < 1e-5) {
      break;
    }

    for (const double& alpha : alphas) {
      auto [xs_new, us_new, j_new] = ForwardPass(alpha, xs, us, ks, Ks);
      all_possible_trajs_.push_back(xs_new);
      if (j_new < j_old) {
        j_old = j_new;
        xs = xs_new;
        us = us_new;
        regu *= 0.7;
        break;
      }
    }
    regu *= 2;
    regu = std::min(std::max(regu, 0.0001), 10000.0);
  }
  return {xs, us};
}
casadi::DM CiLQR::RollOut(const casadi::DM& x0, const casadi::DM& us) {
  casadi::DM xs = casadi::DM::zeros(params_["num_state"], params_["horizon"] + 1);
  xs(casadi::Slice(), 0) = x0;
  for (int i = 0; i < params_["horizon"]; i++) {
    auto input = std::vector<casadi::DM> {xs(casadi::Slice(), i), 
                                          us(casadi::Slice(), i)};
    xs(casadi::Slice(), i + 1) = f_(input)[0];
  } 
  return xs;
}
std::tuple<casadi::DM, casadi::DM, double> CiLQR::ForwardPass(const double& alpha,
                                                              const casadi::DM& xs, 
                                                              const casadi::DM& us, 
                                                              const casadi::DM& ks, 
                                                              const std::vector<casadi::DM>& Ks) {
  casadi::DM xs_new = casadi::DM::zeros(params_["num_state"], params_["horizon"] + 1);
  casadi::DM us_new = us + alpha * ks;

  double cost = 0;
  xs_new(casadi::Slice(), 0) = xs(casadi::Slice(), 0);
  for (int i = 0; i < params_["horizon"]; i++) {
    us_new(casadi::Slice(), i) += mtimes(Ks[i], (xs_new(casadi::Slice(), i) - xs(casadi::Slice(), i)));
    
    auto input = std::vector<casadi::DM> {xs_new(casadi::Slice(), i), 
                                          us_new(casadi::Slice(), i)};
    xs_new(casadi::Slice(), i + 1) = f_(input)[0];
    
    // std::cout << "xs new: " << xs_new(casadi::Slice(), i) << std::endl;
    // std::cout << "us new: " << us_new(casadi::Slice(), i) << std::endl;

    cost += l_(input)[0].scalar();

    // print the single cost 
    // auto&& cost_result = l_debug_(input);
    // std::cout << "state cost: " << cost_result[0] << std::endl;
    // std::cout << "control cost: " << cost_result[1] << std::endl;
    // std::cout << "obs cost: " << cost_result[2] << std::endl;
  }

  return {xs_new, us_new, cost};
}
std::tuple<casadi::DM, std::vector<casadi::DM>, double> CiLQR::BackwardPass(
    const casadi::DM& xs, const casadi::DM& us, double regu) {

  casadi::DM ks = casadi::DM::zeros(params_["num_control"], (params_["horizon"]));
  std::vector<casadi::DM> Ks(params_["horizon"], casadi::DM::zeros(params_["num_control"], 
                                                                   params_["num_state"]));
  casadi::DM v_x = casadi::DM::zeros(params_["num_state"]);
  casadi::DM v_xx = casadi::DM::zeros(params_["num_state"], params_["num_state"]);
  
  double delta_v = 0;
  auto reguI = regu * casadi::DM::eye(params_["num_state"]);
  for (int i = params_["horizon"] - 1; i >= 0; i--) {
    auto input = std::vector<casadi::DM>{xs(casadi::Slice(), i + 1), us(casadi::Slice(), i)};
    auto f_x = f_x_(input)[0];
    auto f_u = f_u_(input)[0];
    auto l_x = l_x_(input)[0];
    auto l_u = l_u_(input)[0];
    auto l_xx = l_xx_(input)[0];
    auto l_uu = l_uu_(input)[0];
    auto l_ux = l_ux_(input)[0];

    auto&& Q_x = l_x + mtimes(f_x.T(), v_x);
    auto&& Q_u = l_u + mtimes(f_u.T(), v_x);
    auto&& Q_xx = l_xx + mtimes(mtimes(f_x.T(), v_xx), f_x);
    auto&& Q_ux = l_ux + mtimes(mtimes(f_u.T(), v_xx), f_x);
    auto&& Q_uu = l_uu + mtimes(mtimes(f_u.T(), v_xx), f_u);

    auto&& f_u_dot_regu = mtimes(f_u.T(), reguI);
    auto&& Q_ux_regu = Q_ux + mtimes(f_u_dot_regu, f_x);
    auto&& Q_uu_regu = Q_uu + mtimes(f_u_dot_regu, f_u);
    auto&& Q_uu_inv = inv(Q_uu_regu);

    // auto&& Q_uu_inv = inv(Q_uu);
    auto&& k = mtimes(-Q_uu_inv, Q_u);
    auto&& K = mtimes(-Q_uu_inv, Q_ux_regu);

    v_x = Q_x + mtimes(mtimes(K.T(), Q_uu), k) + mtimes(K.T(), Q_u) + mtimes(Q_ux.T(), k);
    v_xx = Q_xx + mtimes(mtimes(K.T(), Q_uu), K) + mtimes(K.T(), Q_ux) + mtimes(Q_ux.T(), K);
    
    delta_v += (mtimes(Q_u.T(), k) + 0.5 * mtimes(mtimes(k.T(), Q_uu), k)).scalar();
    // std::cout << "delta v: " << delta_v << std::endl;

    ks(casadi::Slice(), i) = std::move(k);
    Ks[i] = std::move(K);
  }

  return {ks, Ks, delta_v};
}
}
}