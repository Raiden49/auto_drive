/*
 * @Author: Raiden49 
 * @Date: 2024-10-17 09:56:01 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-10-17 16:50:48
 */
#ifndef CILQR_VEHICLE_MODEL_HPP_
#define CILQR_VEHICLE_MODEL_HPP_

#include <casadi/casadi.hpp>

namespace auto_drive {
namespace cilqr_planner{
class VehicleModel {
 public:
  VehicleModel(std::unordered_map<std::string, double>& params) : 
               wheel_base_(params["wheel_base"]), steer_min_(params["steer_min"]), 
               steer_max_(params["steer_max"]), accel_min_(params["acc_min"]), 
               accel_max_(params["acc_max"]), max_speed_(params["max_speed"]),
               delta_t_(params["delta_t"]), N_(params["horizon"]) {};

  casadi::SX VehicleModelConstruct(casadi::SX& state, casadi::SX& control) {
    return ForwardSimulate(state, control);
  }

 public:
  double delta_t_;
  double N_;
  double wheel_base_;
  double steer_min_;
  double steer_max_;
  double accel_min_;
  double accel_max_;
  double max_speed_;

 private:
  casadi::SX ForwardSimulate(casadi::SX& state, casadi::SX& control) {
    // state: x, y, v, yaw
    // control: acc, steer_v

    // x(k + 1) = x(K) + v(k) * cos(yaw(k)) * t + 0.5 * acc(k) * cos(yaw(k)) * t * t
    // y(k + 1) = y(k) + v(k) * sin(yaw(k)) * t + 0.5 * acc(k) * sin(yaw(k)) * t * t;
    // v(k + 1) = v(k) + acc(k) * t;
    // yaw(k + 1) = yaw(k) + steer_v(k) * t
    casadi::SX next_state = casadi::SX::veccat({
        state(0) + cos(state(3)) * (state(2) * delta_t_ + (control(0)* delta_t_ * delta_t_) / 2),
        state(1) + sin(state(3)) * (state(2) * delta_t_ + (control(0)* delta_t_ * delta_t_) / 2),
        fmin(state(2) + control(0) * delta_t_, max_speed_),
        state(3) + control(1) * delta_t_});
    
    return next_state;
  }
};
}
} // namespace auto_drive

#endif // VEHICLE_MODEL_HPP_