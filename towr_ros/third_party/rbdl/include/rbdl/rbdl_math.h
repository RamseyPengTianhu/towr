/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_MATH_H
#define RBDL_MATH_H

#include "rbdl/rbdl_config.h"

#include <Eigen/Dense>
#include <Eigen/QR>

#include "rbdl/rbdl_eigenmath.h"

typedef Eigen::Matrix<double, 6, 3> Matrix63_t;
typedef Eigen::Matrix<double, 4, 3> Matrix43_t;

typedef Eigen::VectorXd VectorN_t;
typedef Eigen::MatrixXd MatrixN_t;

namespace RigidBodyDynamics {

/** \brief Math types such as vectors and matrices and utility functions. */
namespace Math {
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector4d Vector4d;
typedef Eigen::Matrix3d Matrix3d;
typedef SpatialVector_t SpatialVector;
typedef SpatialMatrix_t SpatialMatrix;
typedef Matrix63_t Matrix63;
typedef Matrix43_t Matrix43;
typedef VectorN_t VectorNd;
typedef MatrixN_t MatrixNd;
} /* Math */

} /* RigidBodyDynamics */

#include "rbdl/Quaternion.h"
#include "rbdl/SpatialAlgebraOperators.h"

  /* RBDL_MATH_H_H */
#endif
