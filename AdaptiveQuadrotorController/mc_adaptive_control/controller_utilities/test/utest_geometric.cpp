/**
 * @file utest_geometric.cpp
 * 
 * These unit tests simply ensure our implementation is matching the output
 * of controller from matlab.
 *
 * @author Sam Ubellacker <subella@mit.edu>
 *
 */

#include <gtest/gtest.h>
#include <mc_adaptive_control_utils/geometric_controller.hpp>

#define TOLERANCE 1.0e-2f

using namespace mc_adaptive_control;
using matrix::Dcmf;
using matrix::Vector3f;


// These unit tests simply ensure our implementation is matching the output
// of controller from matlab.

TEST(GeometricPosition, Idle) {
  GeometricPosController controller;
  controller.gains.kx = 1.0;
  controller.gains.kv = 1.0;
  controller.properties.m = 1.0;
  controller.properties.J(0, 0) = 1.0;
  controller.properties.J(1, 1) = 1.0;
  controller.properties.J(2, 2) = 1.0;

  Vector3f pos(1.0f, 2.0f, -0.5f);
  Vector3f vel(0.0f, 0.0f, 0.0f);

  VehicleState state;
  state.pos = pos;
  state.vel = vel;

  struct vehicle_local_position_setpoint_s setpoint;
  setpoint.x = pos(0);
  setpoint.y = pos(1);
  setpoint.z = pos(2) + 1.0f;
  setpoint.yaw = 0.0f;
  setpoint.vx = 0.0f;
  setpoint.vy = 0.0f;
  setpoint.vz = 1.0f;
  setpoint.acceleration[0] = 0.0f;
  setpoint.acceleration[1] = 0.0f;
  setpoint.acceleration[2] = 0.0f;

  struct vehicle_attitude_setpoint_s output;
  controller.run(state, setpoint, output);

  // attitude setpoint
  EXPECT_NEAR(output.q_d[0], 1.0f, TOLERANCE);
  EXPECT_NEAR(output.q_d[1], 0.0f, TOLERANCE);
  EXPECT_NEAR(output.q_d[2], 0.0f, TOLERANCE);
  EXPECT_NEAR(output.q_d[3], 0.0f, TOLERANCE);
  // derivative derived setpoints
  EXPECT_NEAR(output.angle_vel_x, 0.0f, TOLERANCE);
  EXPECT_NEAR(output.angle_vel_y, 0.0f, TOLERANCE);
  EXPECT_NEAR(output.angle_vel_z, 0.0f, TOLERANCE);
  EXPECT_NEAR(output.angle_acc_x, 0.0f, TOLERANCE);
  EXPECT_NEAR(output.angle_acc_y, 0.0f, TOLERANCE);
  EXPECT_NEAR(output.angle_acc_z, 0.0f, TOLERANCE);
  // thrust magnitude
  EXPECT_NEAR(output.thrust_body[2], 7.81f, TOLERANCE);
}

TEST(GeometricPosition, PositionSetpoint) {
  GeometricPosController position_controller;
  position_controller.gains.kx = 12.0;
  position_controller.gains.kv = 8.0;
  position_controller.gains.gamma_x = 2;
  position_controller.gains.c1 = 2.4;
  position_controller.properties.m = 2.0;
  position_controller.properties.J(0, 0) = 0.02;
  position_controller.properties.J(1, 1) = 0.02;
  position_controller.properties.J(2, 2) = 0.04;
  position_controller.adaptive_params.use_adaptive_term = true;
  position_controller._bar_theta_x = Vector3f (1.0f, 2.0f, 3.0f);

  Vector3f pos(1.0f, -1.0f, 0.0f);
  Vector3f vel(0.0f, 0.0f, 0.0f);
  
  VehicleState state;
  state.pos = pos;
  state.vel = vel;
  state.R = matrix::eye<float, 3>(); 

  struct vehicle_local_position_setpoint_s setpoint;
  setpoint.x = -1.309f;
  setpoint.y = 0.9511f;
  setpoint.z = -1.0f;
  setpoint.yaw = 0.0f;
  setpoint.vx = -0.5976f;
  setpoint.vy = -0.1946f;
  setpoint.vz = 0.0f;
  setpoint.acceleration[0] = 0.122f;
  setpoint.acceleration[1] = -0.3755f;
  setpoint.acceleration[2] = 0.0f;

  struct vehicle_attitude_setpoint_s attitude_setpoint;
  position_controller.run(state, setpoint, attitude_setpoint);

  EXPECT_NEAR(position_controller._A(0), -33.2447f, TOLERANCE);
  EXPECT_NEAR(position_controller._A(1), 19.1085f, TOLERANCE);
  EXPECT_NEAR(position_controller._A(2), -34.62f, TOLERANCE);
  EXPECT_NEAR(position_controller._A_dot(0), -22.4733f, TOLERANCE);
  EXPECT_NEAR(position_controller._A_dot(1), -4.3569f, TOLERANCE);
  EXPECT_NEAR(position_controller._A_dot(2), 43.2f, TOLERANCE);
  EXPECT_NEAR(position_controller._A_ddot(0), -57.274f, TOLERANCE);
  EXPECT_NEAR(position_controller._A_ddot(1), 15.7186f, TOLERANCE);
  EXPECT_NEAR(position_controller._A_ddot(2), -108.0f, TOLERANCE);
    
  GeometricAttController attitude_controller;
  attitude_controller.gains.kr = 6.0;
  attitude_controller.gains.komega = 2.0;
  attitude_controller.gains.gamma_r = 10;
  attitude_controller.gains.c2 = 10.5254;
  attitude_controller.properties.m = 2.0;
  attitude_controller.properties.J(0, 0) = 0.02;
  attitude_controller.properties.J(1, 1) = 0.02;
  attitude_controller.properties.J(2, 2) = 0.04;
  attitude_controller.adaptive_params.use_adaptive_term = true;
  attitude_controller._bar_theta_r = Vector3f (1.0f, 2.0f, 3.0f);
  
  matrix::Vector4f output;
  attitude_controller.run(state, attitude_setpoint, output);
  EXPECT_NEAR(output(0), -0.2212f, TOLERANCE);
  EXPECT_NEAR(output(1), -2.6165f, TOLERANCE);
  EXPECT_NEAR(output(2), 2.4742f, TOLERANCE);
  EXPECT_NEAR(output(2), 0.0f, TOLERANCE);

}
