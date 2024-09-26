/*
 * @Author: Raiden49 
 * @Date: 2024-09-24 09:37:21 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 15:31:00
 */
#ifndef QUINTIC_POLYNOMIAL_HPP_
#define QUINTIC_POLYNOMIAL_HPP_

#include <Eigen/Eigen>
#include <cmath>

class QuinticPolynomial {
 public:
  QuinticPolynomial() = delete;
  /**
   * @brief Construct a new Quintic Polynomial object 构建五次多项式类
   * 
   * @param xs 起点坐标
   * @param vs 起点速度
   * @param as 起点加速度
   * @param xe 终点坐标
   * @param ve 终点速度
   * @param ae 终点加速度
   * @param Ts 起始时刻
   * @param Te 终止时刻
   */
  QuinticPolynomial(const double& xs, const double& vs, const double& as, 
                    const double& xe, const double& ve, const double& ae,
                    const double& Ts, const double& Te) {
    Eigen::MatrixXd A(6, 6);
    A << 1, Ts, pow(Ts, 2), pow(Ts, 3), pow(Ts, 4), pow(Ts, 5),
         0, 1, 2 * Ts, 3 * pow(Ts, 2), 4 * pow(Ts, 3), 5 * pow(Ts, 4),
         0, 0, 2, 6 * Ts, 12 * pow(Ts, 2), 20 * pow(Ts, 3),
         1, Te, pow(Te, 2), pow(Te, 3), pow(Te, 4), pow(Te, 5),
         0, 1, 2 * Te, 3 * pow(Te, 2), 4 * pow(Te, 3), 5 * pow(Te, 4),
         0, 0, 2, 6 * Te, 12 * pow(Te, 2), 20 * pow(Te, 3);
    Eigen::VectorXd B(6);
    B << xs, vs, as, xe, ve, ae;

    Eigen::VectorXd coeff(6);
    coeff = A.colPivHouseholderQr().solve(B);
    a0 = coeff[0]; a1 = coeff[1]; a2 = coeff[2];
    a3 = coeff[3]; a4 = coeff[4]; a5 = coeff[5];
  }

  inline double CalPoint(const double& t) {
    return a0 + a1 * t + a2 * pow(t, 2) + 
           a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
  }
  inline double CalFirstDerivative(const double& t) {
    return a1 + 2 * a2 * t + 
           3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
  }
  inline double CalSecondDerivative(const double& t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
  }
  inline double CalThirdDerivative(const double& t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
  }

 public:
  double a0, a1, a2, a3, a4, a5;
};

#endif // QUINTIC_POLYNOMIAL_HPP_