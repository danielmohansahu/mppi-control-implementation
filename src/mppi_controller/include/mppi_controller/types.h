/* types.h
 *
 * Common definitions used across this package.
 */

#pragma once

#include <eigen3/Eigen/Core>

namespace mppi
{

// state is represented by (x, y, theta, dx, dy, dtheta)
constexpr size_t STATE_DIM = 6;

// position is represented by (x, y, theta)
constexpr size_t POS_DIM = 3;
constexpr size_t X_IDX = 0;
constexpr size_t Y_IDX = 1;
constexpr size_t TH_IDX = 2;

// velocity is represented by (dx, dy, dtheta)
constexpr size_t VEL_DIM = 3;
constexpr size_t DX_IDX = 3;
constexpr size_t DY_IDX = 4;
constexpr size_t DTH_IDX = 5;

// control is represented by (left wheel angular vel, right wheel angular vel)
constexpr size_t CONTROL_DIM = 2;

// Array shorthands
using Statef    = Eigen::Matrix<float, STATE_DIM, 1>;
using Posef     = Eigen::Matrix<float, POS_DIM, 1>;
using Velf      = Eigen::Matrix<float, VEL_DIM, 1>;
using Controlf  = Eigen::Matrix<float, CONTROL_DIM, 1>;

// convenience typedefs of common types
using Matrix    = Eigen::MatrixXf;
using VelMatrix = Eigen::Matrix<float, VEL_DIM, CONTROL_DIM>;

} // namespace mppi
