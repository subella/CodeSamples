/**
 * @file transform_utils.cpp
 * Quick lie-group operators
 *
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 * @author Sam Ubellacker <subella@mit.edu>
 *
 */

#include <mc_adaptive_control_utils/transform_utils.hpp>

using matrix::Matrix3f;
using matrix::Vector3f;

namespace mc_adaptive_control {

Vector3f Vee(Matrix3f input) {
  Vector3f output(input(2, 1), input(0, 2), input(1, 0));
  return output;
};

Matrix3f Hat(Vector3f input) {
  Matrix3f out;

  out(0, 0) = 0.0f;
  out(0, 1) = -input(2);
  out(0, 2) = input(1);

  out(1, 0) = input(2);
  out(1, 1) = 0.0f;
  out(1, 2) = -input(0);

  out(2, 0) = -input(1);
  out(2, 1) = input(0);
  out(2, 2) = 0.0f;

  return out;
};

}  // namespace mc_adaptive_control
