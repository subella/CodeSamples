/**
 * @file geometric_controller.cpp
 * Implementation of a geometric multicopter control law
 *
 * @author Sam Ubellacker <subella@mit.edu>
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 *
 */
#include <limits.h>
#include <mc_adaptive_control_utils/geometric_controller.hpp>
#include <mc_adaptive_control_utils/transform_utils.hpp>

namespace mc_adaptive_control {

using matrix::Dcmf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::Vector4f;

typedef struct vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
typedef struct vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

GeometricAttController::GeometricAttController() { 
  properties.J.zero();
  _bar_theta_r.zero();
  _W_r = matrix::eye<float, 3>();
}

void GeometricAttController::run(const VehicleState& state,
                                 const vehicle_attitude_setpoint& input,
                                 Vector4f& output) {
  if (!adaptive_params.use_adaptive_term) {
    _W_r.zero();
  } else {
    _W_r = matrix::eye<float, 3>();
  }

  Quatf q_d(input.q_d);
  Dcmf Rd(q_d.normalized());
  Vector3f omega_d(input.angle_vel_x, input.angle_vel_y, input.angle_vel_z);
  Vector3f omegad_dot(input.angle_acc_x, input.angle_acc_y, input.angle_acc_z);
  Matrix3f R_inv = state.R.transpose();
  _er = 0.5f * Vee(Rd.transpose() * state.R - R_inv * Rd);
  _eomega = state.omega - R_inv * omega_d;

  Vector3f M = -gains.kr * _er - gains.komega * _eomega -
               _W_r * _bar_theta_r +
               Hat(R_inv * omega_d) * properties.J * R_inv * omega_d +
               properties.J * R_inv * omegad_dot;

  Vector3f bar_theta_r_dot = gains.gamma_r * _W_r.transpose() * (_eomega + gains.c2 * _er);
   _bar_theta_r += bar_theta_r_dot * state.dt;

  output(0) = M(0);
  output(1) = M(1);
  output(2) = M(2);
  output(3) = input.thrust_body[2];
}

GeometricPosController::GeometricPosController() {
  // TODO(nathan) grab the G constant from somewhere else maybe
  _g = Vector3f(0, 0, 9.81);
  // TODO(sam) expose W matrix as parameter if needed
  _bar_theta_x.zero();
  _W_x = matrix::eye<float, 3>();
  properties.J.zero();
  _last_ev.zero();
}

auto GeometricPosController::getRd() const -> Matrix3f { return _Rd; }

auto GeometricPosController::getRd_dot() const -> Matrix3f { return _Rd_dot; }

auto GeometricPosController::getRd_ddot() const -> Matrix3f { return _Rd_ddot; }

void GeometricPosController::run(const VehicleState& state,
                                 const vehicle_local_position_setpoint& input,
                                 vehicle_attitude_setpoint& output) {

  if (!adaptive_params.use_adaptive_term) {
    _W_x.zero();
  } else {
    _W_x = matrix::eye<float, 3>();
  }

  Vector3f goal_pos(input.x, input.y, input.z);
  Vector3f goal_vel(input.vx, input.vy, input.vz);
  Vector3f goal_acc(input.acceleration[0], input.acceleration[1], input.acceleration[2]);

  _ex = state.pos - goal_pos;
  _ev = state.vel - goal_vel;

  _A = -gains.kx * _ex - gains.kv * _ev + properties.m * (goal_acc - _g) -
       _W_x * _bar_theta_x;

  Vector3f ev_c1ex = _ev + gains.c1 * _ex;
  Vector3f bar_theta_x_dot = gains.gamma_x * _W_x.transpose() * ev_c1ex;

  Vector3f e3 = Vector3f(0, 0, 1);
  Vector3f b3 = state.R * e3;
  float f = -_A * b3;
  Vector3f ev_dot = _g - f/properties.m * b3 - goal_acc + _W_x * _bar_theta_x / properties.m;

  _A_dot = -gains.kx * _ev - gains.kv * ev_dot - _W_x * bar_theta_x_dot;

  Vector3f bar_theta_x_2dot = gains.gamma_x * _W_x.transpose() * (ev_dot + gains.c1 * _ev);

  Vector3f b3_dot = state.R * Hat(state.omega) * e3;
  float f_dot = - _A_dot * b3 - _A * b3_dot;
  Vector3f ev_2dot = - f_dot / properties.m * b3 - f / properties.m * b3_dot + 
                     _W_x * bar_theta_x_dot / properties.m;
  _A_ddot = -gains.kx * ev_dot - gains.kv * ev_2dot - _W_x * bar_theta_x_2dot;
  _bar_theta_x += bar_theta_x_dot * state.dt;

  _get_rotation_and_derivatives(input.yaw);
  _write_to_output(state, output);
}

void GeometricPosController::_write_to_output(const VehicleState& state,
                                             vehicle_attitude_setpoint& output) {
  Vector3f omega_d = _Rd * Vee(_Rd.transpose() * _Rd_dot);
  Vector3f omegad_dot =
      _Rd * Vee(_Rd.transpose() * _Rd_ddot - Hat(omega_d) * Hat(omega_d));

  // copy output to uorb message
  Dcmf R_sp(_Rd);
  Quatf q_sp(R_sp);
  q_sp.copyTo(output.q_d);

  output.thrust_body[2] = (-_A).dot(state.R.col(2));

  output.angle_vel_x = omega_d(0);
  output.angle_vel_y = omega_d(1);
  output.angle_vel_z = omega_d(2);

  output.angle_acc_x = omegad_dot(0);
  output.angle_acc_y = omegad_dot(1);
  output.angle_acc_z = omegad_dot(2);
}

void GeometricPosController::_derive_unit_vector(Vector3f& q, Vector3f& q_dot, Vector3f& q_ddot,
                                                 Vector3f& u, Vector3f& u_dot, Vector3f& u_ddot){
    float nq = q.norm();
    u = q / nq;
    u_dot = q_dot / nq - q * q.dot(q_dot) / powf(nq, 3);

    u_ddot = q_ddot / nq - q_dot / powf(nq, 3) * (2 * q.dot(q_dot)) -
             q / powf(nq, 3) * (q_dot.dot(q_dot) + q.dot(q_ddot)) +
             3.0f * q / powf(nq, 5) * powf(q.dot(q_dot), 2);
}

void GeometricPosController::_get_rotation_and_derivatives(double yaw_d) {
    
  Vector3f q = -_A;
  Vector3f q_dot = -_A_dot;
  Vector3f q_ddot = -_A_ddot;
  Vector3f b3c, b3c_dot, b3c_ddot;
  _derive_unit_vector(q, q_dot, q_ddot, b3c, b3c_dot, b3c_ddot);
  
  const Vector3f b1d(cos(yaw_d), sin(yaw_d), 0.0f);

  Vector3f A2 = -Hat(b1d) * b3c;
  Vector3f A2_dot = -Hat(b1d) * b3c_dot;
  Vector3f A2_ddot = -Hat(b1d) * b3c_ddot;
  Vector3f b2c, b2c_dot, b2c_ddot;
  _derive_unit_vector(A2, A2_dot, A2_ddot, b2c, b2c_dot, b2c_ddot);

  Vector3f b1c = Hat(b2c) * b3c;
  Vector3f b1c_dot = Hat(b2c_dot) * b3c + Hat(b2c) * b3c_dot;
  Vector3f b1c_ddot = Hat(b2c_ddot) * b3c + 2.0f * Hat(b2c_dot) * b3c_dot +
                      Hat(b2c) * b2c_ddot;

  _Rd.setCol(0, b1c);
  _Rd.setCol(1, b2c);
  _Rd.setCol(2, b3c);

  _Rd_dot.setCol(0, b1c_dot);
  _Rd_dot.setCol(1, b2c_dot);
  _Rd_dot.setCol(2, b3c_dot);

  _Rd_ddot.setCol(0, b1c_ddot);
  _Rd_ddot.setCol(1, b2c_ddot);
  _Rd_ddot.setCol(2, b3c_ddot);
}

}  // namespace mc_adaptive_control
