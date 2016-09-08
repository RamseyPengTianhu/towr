/**
@file   com_polynomial.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Declares CoeffValues, PolynomialFifthOrder and ComPolynomial
 */

#ifndef _XPP_ZMP_COM_POLYNOMIAL_H_
#define _XPP_ZMP_COM_POLYNOMIAL_H_

#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace zmp {

static const int kCoeffCount = 6;
enum SplineCoeff { A=0, B, C, D, E, F };

struct CoeffValues {
  double x[kCoeffCount];
  double y[kCoeffCount];
  CoeffValues()
  {
    for (int c = A; c <= F; ++c)
      x[c] = y[c] = 0.0;
  };

  CoeffValues(double xa, double xb, double xc, double xd, double xe, double xf,
              double ya, double yb, double yc, double yd, double ye, double yf)
  {
    x[A] = xa; x[B] = xb; x[C] = xc; x[D] = xd; x[E] = xe; x[F] = xf;
    y[A] = ya; y[B] = yb; y[C] = yc; y[D] = yd; y[E] = ye; y[F] = yf;
  }

  /** generates random spline coefficients between -25 and 25 */
  void SetRandom()
  {
    for (int c = A; c <= F; ++c) {
      x[c] = (double)rand() / RAND_MAX * 50 - 25;
      y[c] = (double)rand() / RAND_MAX * 50 - 25;
    }
  }
};


/** Builds a 5th order polynomial from the polynomial coefficients.
  */
class PolynomialFifthOrder {
public:
  typedef xpp::utils::Vec2d Vec2d;
  typedef xpp::utils::MotionDerivative PosVelAcc;

  PolynomialFifthOrder();
  PolynomialFifthOrder(const CoeffValues &coeff_values);
  virtual ~PolynomialFifthOrder() {};

  Vec2d GetState(PosVelAcc whichDeriv, double t) const;
  void SetSplineCoefficients(const CoeffValues &coeff_values = CoeffValues());
  double GetCoefficient(int dim, SplineCoeff coeff) const;

private:
  static constexpr int kDim2d = xpp::utils::kDim2d;

protected:
  double spline_coeff_[kDim2d][kCoeffCount];
  friend class SplineContainerTest_EandFCoefficientTest_Test;

};

} // namespace zmp
} // namespace xpp

namespace xpp {namespace ros{ class RosHelpers; }};

namespace xpp {
namespace zmp {

/** A fifth order spline that now holds some context information.
  */
class ComPolynomial : public PolynomialFifthOrder {
public:
  ComPolynomial();
  ComPolynomial(uint id, double duration);
  virtual ~ComPolynomial() {};

  uint GetId()            const { return id_; };
  double GetDuration()    const { return duration_; };

private:
  uint id_; // to identify the order relative to other polynomials
  double duration_; // time during which this spline is active

  friend std::ostream& operator<<(std::ostream& out, const ComPolynomial& tr);
  friend struct xpp::ros::RosHelpers;
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_COM_POLYNOMIAL_H_
