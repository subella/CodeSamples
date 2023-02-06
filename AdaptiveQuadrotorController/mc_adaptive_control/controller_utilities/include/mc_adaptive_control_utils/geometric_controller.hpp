/*
 * @file geometric_controller.hpp
 * Implementation of a geometric multicopter control law
 *
 * @author Sam Ubellacker <subella@mit.edu>
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 *
 */

#pragma once
#include <mc_adaptive_control_utils/control_config_structures.hpp>
#include <mc_adaptive_control_utils/matrix_types.hpp>

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

namespace mc_adaptive_control {

class GeometricAttController {
 public:
  GeometricAttController();

  virtual ~GeometricAttController() = default;

  virtual void run(const VehicleState& state,
                   const struct vehicle_attitude_setpoint_s& input,
                   matrix::Vector4f& output);

  VehicleProperties properties;
  AttitudeGains gains;
  AdaptiveParams adaptive_params;

 public:
  matrix::Vector3f _er;
  matrix::Vector3f _eomega;
  matrix::Vector3f _bar_theta_r;
  matrix::Matrix3f _W_r;
};

class GeometricPosController {
 public:
  GeometricPosController();

  virtual ~GeometricPosController() = default;

  virtual void run(const VehicleState& state,
                   const struct vehicle_local_position_setpoint_s& input,
                   struct vehicle_attitude_setpoint_s& output);

  VehicleProperties properties;
  PositionGains gains;
  AdaptiveParams adaptive_params;

  auto getRd() const -> matrix::Matrix3f;
  auto getRd_dot() const -> matrix::Matrix3f;
  auto getRd_ddot() const -> matrix::Matrix3f;

 public:
  virtual void _write_to_output(const VehicleState& state, struct vehicle_attitude_setpoint_s& output);
  virtual void _derive_unit_vector(matrix::Vector3f& q, matrix::Vector3f& q_dot, matrix::Vector3f& q_ddot,
                                   matrix::Vector3f& u, matrix::Vector3f& u_dot, matrix::Vector3f& u_ddot);
  virtual void _get_rotation_and_derivatives(double yaw_d);

  matrix::Vector3f _ex;
  matrix::Vector3f _ev;
  matrix::Vector3f _g;
  matrix::Vector3f _bar_theta_x;
  matrix::Matrix3f _W_x;
  float _B_theta_x;
  matrix::Vector3f _A;
  matrix::Vector3f _A_dot;
  matrix::Vector3f _A_ddot;
  matrix::Vector3f _last_ev;
  matrix::Dcmf _Rd;
  matrix::Matrix3f _Rd_dot;
  matrix::Matrix3f _Rd_ddot;
};

}  // namespace mc_adaptive_control
