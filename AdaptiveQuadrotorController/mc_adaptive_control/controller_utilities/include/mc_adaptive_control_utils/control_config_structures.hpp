/*
 * @file control_config_structures.hpp
 * Configuration structures for the various controllers
 *
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 * @author Samuel Ubellacker <subella@mit.edu>
 *
 */
#pragma once
#include <mc_adaptive_control_utils/matrix_types.hpp>

namespace mc_adaptive_control {

enum class AdotMode : int {
  LEGACY = 0,
  FINITE_DIFFERENCES = 1,
  IGNORE_ACCEL = 2,
};

struct PositionGains {
  float kx;
  float kv;
  float c1;
  float gamma_x;
  AdotMode mode;
};

struct AttitudeGains {
  float kr;
  float komega;
  float c2;
  float gamma_r;
};

struct AdaptiveParams {
  float limit;
  float B_theta_x;
  bool run_update{false};
  bool use_adaptive_term{false};
};

struct VehicleProperties {
  float m;
  matrix::Matrix3f J;
};

struct VehicleState {
  matrix::Dcmf R;
  matrix::Vector3f omega;
  matrix::Vector3f pos;
  matrix::Vector3f vel;
  matrix::Vector3f acc;
  float dt;
  float motor_pwm_values[4];
};
}  // namespace mc_adaptive_control
