/*
 * @Author: Raiden49 
 * @Date: 2024-09-29 10:20:44 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-29 10:50:43
 */
#ifndef QUARTIC_POLYNOMIAL_HPP_
#define QUARTIC_POLYNOMIAL_HPP_

#include <Eigen/Eigen>
#include <cmath>

namespace auto_drive {
// p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
class QuarticPolynomial {
 public:
  QuarticPolynomial() = delete;
  QuarticPolynomial(const double& xs, const double& vs, const double& as,
                    const double& ve, const double& ae, const double& t) : 
                    xs_(xs), vs_(vs), as_(as), ve_(ve), ae_(ae) {
    a0_ = xs; a1_ = vs; a2_ = as / 2.0;
    Eigen::Matrix2f A;
    A << 3 * pow(t, 2), 4 * pow(t, 3), 6 * t, 12 * pow(t, 2);
    Eigen::Vector2f B;
    B << ve - a1_ - 2 * a2_ * t, ae_ - 2 * a2_;

    // d(p(t)) = ve, d(d(p(t))) = ae;
    Eigen::Vector2f coeff = A.colPivHouseholderQr().solve(B);
    a3_ = coeff[0]; a4_ = coeff[1];
  }

  double CalPoint(const double& t) {
    return a0_ + a1_ * t + a2_ * pow(t, 2) + a3_ * pow(t, 3) + a4_ * pow(t, 4);
  }
  double CalFirstDerivative(const double& t) {
    return a1_ + 2 * a2_ * t + 3 * a3_ * pow(t, 2) + 4 * a4_ * pow(t, 3);
  }
  double CalSecondDerivative(const double& t) {
    return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * pow(t, 2);
  }
  double CalThirdDerivative(const double& t) {
    return 6 * a3_ + 24 * a4_ * t;
  }

 public:
  double xs_, vs_, as_, ve_, ae_;
  double a0_, a1_, a2_, a3_, a4_;
};
}

#endif // QUARTIC_POLYNOMIAL_HPP_