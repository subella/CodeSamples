/**
 * @file transform_utils.hpp
 * Quick lie-group operators
 *
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 * @author Sam Ubellacker <subella@mit.edu>
 *
 */
#pragma once
#include <matrix/matrix/math.hpp>

namespace mc_adaptive_control {

matrix::Vector3f Vee(matrix::Matrix3f input);

matrix::Matrix3f Hat(matrix::Vector3f input);

}  // namespace mc_adaptive_control
