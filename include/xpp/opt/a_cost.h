/**
 @file    a_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a cost for the NLP problem.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_COST_H_

#include <Eigen/Dense>
#include "variable_names.h"

namespace xpp {
namespace opt {

class OptimizationVariables;

/** @brief Common interface to define a cost, which simply returns a scalar value */
class ACost {
public:
  using VectorXd = Eigen::VectorXd;

  ACost () {};
  virtual ~ACost () {}

  virtual double EvaluateCost () const = 0;
  virtual void UpdateVariables(const OptimizationVariables*) = 0;
  virtual VectorXd EvaluateGradientWrt(std::string var_set) { assert(false); };// motion_ref remove default implementation
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_COST_H_ */