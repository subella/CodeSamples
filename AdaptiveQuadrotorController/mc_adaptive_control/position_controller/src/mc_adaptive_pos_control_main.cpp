/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_pos_control_main.cpp
 * Adaptive multicopter position controller.
 *
 * HEAVILY refactored position controller that more closely follows
 * the original geometric control paper.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 */
#include <mc_adaptive_control_utils/px4_helpers.hpp>
#include <mc_adaptive_pos_control/mc_adaptive_pos_control.hpp>

namespace mc_adaptive_control {

AdaptiveMulticopterPositionControl::AdaptiveMulticopterPositionControl()
    : ModuleParams(nullptr),
      UorbPoll(20),
      PeriodicLog(500000),
      _loop_perf(perf_alloc(PC_ELAPSED, "mc_adapt_pos_control")),
      _prev_setpoint(SetpointType::NONE),
      _is_idle{true},
      _vehicle_attitude_sub(-1),
      _control_mode_sub(-1),
      _params_sub(-1),
      _manual_sub(-1),
      _local_pos_sub(-1),
      _pos_sp_triplet_sub(-1),
      _actuator_outputs_sub(-1),
      _att_sp_pub(nullptr),
      _local_pos_sp_pub(nullptr),
      _vehicle_status{},
      _vehicle_land_detected{},
      _att{},
      _manual{},
      _control_mode{},
      _param_update{},
      _local_pos{},
      _pos_sp_triplet{},
      _att_sp{},
      _local_pos_sp{} {
  // Set pose information to something sane
  _att.q[0] = 1.0f;
  _R.identity();
  _yaw = 0.0f;
  _pos.zero();
  _vel.zero();
  _acc.zero();

  _update_parameters(true);
}

void AdaptiveMulticopterPositionControl::_update_parameters(bool force) {
  if (force or
      utils::check_and_fill(ORB_ID(parameter_update), _params_sub, &_param_update)) {
    updateParams();
    _position_controller.gains.kx = _position_gain.get();
    _position_controller.gains.kv = _velocity_gain.get();
    _position_controller.gains.c1 = _c1_gain.get();
    _position_controller.gains.gamma_x = _gamma_x_gain.get();
    _position_controller.gains.mode = static_cast<AdotMode>(_adot_mode.get());
    _position_controller.adaptive_params.B_theta_x = _B_theta_x.get();
    _position_controller.properties.m = _vehicle_mass.get();
    _position_controller.properties.J(0, 0) = _vehicle_Jxx.get();
    _position_controller.properties.J(1, 1) = _vehicle_Jyy.get();
    _position_controller.properties.J(2, 2) = _vehicle_Jzz.get();
    set_active(_log_enable.get());
  }
}

void AdaptiveMulticopterPositionControl::_update_from_subscriptions() {
  utils::check_and_fill(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
  utils::check_and_fill(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
  utils::check_and_fill(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub,
                        &_vehicle_land_detected);
  utils::check_and_fill(ORB_ID(vehicle_control_mode), _control_mode_sub,
                        &_control_mode);
  utils::check_and_fill(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub,
                        &_pos_sp_triplet);
  utils::check_and_fill(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);
  utils::check_and_fill(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
  utils::check_and_fill(ORB_ID(actuator_outputs), _actuator_outputs_sub,
                        &_actuator_outputs);
}

void AdaptiveMulticopterPositionControl::_initialize_subscriptions() {
  _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
  _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
  _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
  _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
  _params_sub = orb_subscribe(ORB_ID(parameter_update));
  _manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
  _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
  _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
  _actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));
}

void AdaptiveMulticopterPositionControl::_cleanup_subscriptions() {
  orb_unsubscribe(_vehicle_status_sub);
  orb_unsubscribe(_vehicle_land_detected_sub);
  orb_unsubscribe(_vehicle_attitude_sub);
  orb_unsubscribe(_control_mode_sub);
  orb_unsubscribe(_control_mode_sub);
  orb_unsubscribe(_params_sub);
  orb_unsubscribe(_manual_sub);
  orb_unsubscribe(_local_pos_sub);
  orb_unsubscribe(_pos_sp_triplet_sub);
}

void AdaptiveMulticopterPositionControl::_publish_local_position_setpoint() {
  _local_pos_sp.timestamp = hrt_absolute_time();

  if (_local_pos_sp_pub != nullptr) {
    orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub,
                &_local_pos_sp);
  } else {
    _local_pos_sp_pub =
        orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
  }
}

void AdaptiveMulticopterPositionControl::_update_pose_information() {
  // TODO(nathan) finite and staleness checks
  matrix::Quatf q_curr(_att.q);
  _R = matrix::Dcmf(q_curr);
  _yaw = matrix::Eulerf(q_curr).psi();
  if (_local_pos.xy_valid && _local_pos.z_valid) {
    _pos(0) = _local_pos.x;
    _pos(1) = _local_pos.y;
    _pos(2) = _local_pos.z;
  }
  if (_local_pos.v_xy_valid && _local_pos.v_z_valid) {
    _vel(0) = _local_pos.vx;
    _vel(1) = _local_pos.vy;
    _vel(2) = _local_pos.vz;
  }
  _acc(0) = _local_pos.ax;
  _acc(1) = _local_pos.ay;
  _acc(2) = _local_pos.az;
}

void AdaptiveMulticopterPositionControl::_cache_pose() {
  SetpointType curr_setpoint = _get_setpoint_type();
  if (curr_setpoint != _prev_setpoint or curr_setpoint == SetpointType::NONE) {
    _cached_pos = _pos;
    _cached_yaw = _yaw;
  }
}

auto AdaptiveMulticopterPositionControl::_get_setpoint_type() -> SetpointType {
  if (_control_mode.flag_control_manual_enabled) {
    return SetpointType::IDLE;
  }

  if (not _control_mode.flag_control_offboard_enabled) {
    return SetpointType::NONE;
  }

  if (not _pos_sp_triplet.current.valid) {
    return SetpointType::NONE;
  }

  switch (_pos_sp_triplet.current.type) {
    case position_setpoint_s::SETPOINT_TYPE_POSITION:
    case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
    case position_setpoint_s::SETPOINT_TYPE_OFFBOARD:
    case position_setpoint_s::SETPOINT_TYPE_GRASP_TRAJECTORY:
    case position_setpoint_s::SETPOINT_TYPE_GRASP_TRAJECTORY_NO_GE:
      return SetpointType::POSE_TRIPLET;
    case position_setpoint_s::SETPOINT_TYPE_LOITER:
      return SetpointType::HOLD;
    case position_setpoint_s::SETPOINT_TYPE_TAKEOFF:
      return SetpointType::TAKEOFF;
    case position_setpoint_s::SETPOINT_TYPE_LAND:
      return SetpointType::LAND;
    case position_setpoint_s::SETPOINT_TYPE_IDLE:
      return SetpointType::IDLE;
    default:
      return SetpointType::NONE;
  }
}

void AdaptiveMulticopterPositionControl::_fill_setpoint_hold() {
  // TODO(nathan) probably want to check pos validity
  _local_pos_sp.x = _pos_sp_triplet.current.x;
  _local_pos_sp.y = _pos_sp_triplet.current.y;
  _local_pos_sp.z = _pos_sp_triplet.current.z;
  _local_pos_sp.vx = 0.0f;
  _local_pos_sp.vy = 0.0f;
  _local_pos_sp.vz = 0.0f;
  _local_pos_sp.acceleration[0] = 0.0f;
  _local_pos_sp.acceleration[1] = 0.0f;
  _local_pos_sp.acceleration[2] = 0.0f;
  if (_pos_sp_triplet.current.yaw_valid) {
    _local_pos_sp.yaw = _pos_sp_triplet.current.yaw;
  } else {
    _local_pos_sp.yaw = _yaw;
  }
  _local_pos_sp.yawspeed = NAN;
}

void AdaptiveMulticopterPositionControl::_fill_setpoint_takeoff() {
  _local_pos_sp.x = _cached_pos(0);
  _local_pos_sp.y = _cached_pos(1);
  _local_pos_sp.z = _pos_sp_triplet.current.z;
  _local_pos_sp.vx = 0.0f;
  _local_pos_sp.vy = 0.0f;
  _local_pos_sp.vz = 0.5f * _vel(2);
  _local_pos_sp.acceleration[0] = 0.0f;
  _local_pos_sp.acceleration[1] = 0.0f;
  _local_pos_sp.acceleration[2] = 0.0f;
  _local_pos_sp.yaw = _cached_yaw;
  _local_pos_sp.yawspeed = NAN;
}

void AdaptiveMulticopterPositionControl::_fill_setpoint_land() {
  if (_pos_sp_triplet.current.position_valid) {
    _local_pos_sp.x = _pos_sp_triplet.current.x;
    _local_pos_sp.y = _pos_sp_triplet.current.y;
  } else {
    _local_pos_sp.x = _cached_pos(0);
    _local_pos_sp.y = _cached_pos(1);
  }
  _local_pos_sp.z = _pos(2);
  _local_pos_sp.vx = 0.0f;
  _local_pos_sp.vy = 0.0f;
  _local_pos_sp.vz = _land_speed.get();
  _local_pos_sp.acceleration[0] = 0.0f;
  _local_pos_sp.acceleration[1] = 0.0f;
  _local_pos_sp.acceleration[2] = 0.0f;
  _local_pos_sp.yaw = _cached_yaw;
  _local_pos_sp.yawspeed = NAN;
}

void AdaptiveMulticopterPositionControl::_fill_setpoint_idle() {
  _is_idle = true;
  _local_pos_sp.x = _pos(0);
  _local_pos_sp.y = _pos(1);
  _local_pos_sp.z = _pos(2) + 1.0f;
  _local_pos_sp.vx = 0.0f;
  _local_pos_sp.vy = 0.0f;
  _local_pos_sp.vz = 1.0f;
  _local_pos_sp.acceleration[0] = 0.0f;
  _local_pos_sp.acceleration[1] = 0.0f;
  _local_pos_sp.acceleration[2] = 0.0f;
  _local_pos_sp.yaw = _yaw;
  _local_pos_sp.yawspeed = NAN;
}

void AdaptiveMulticopterPositionControl::_fill_setpoint_pose_triplet() {
  if (_pos_sp_triplet.current.position_valid) {
    _local_pos_sp.x = _pos_sp_triplet.current.x;
    _local_pos_sp.y = _pos_sp_triplet.current.y;
    _local_pos_sp.z = _pos_sp_triplet.current.z;
  } else {
    _local_pos_sp.x = _pos(0);
    _local_pos_sp.y = _pos(1);
    _local_pos_sp.z = _pos(2);
  }

  if (_pos_sp_triplet.current.velocity_valid) {
    switch (_pos_sp_triplet.current.velocity_frame) {
      case position_setpoint_s::VELOCITY_FRAME_LOCAL_NED:
        _local_pos_sp.vx = _pos_sp_triplet.current.vx;
        _local_pos_sp.vy = _pos_sp_triplet.current.vy;
        _local_pos_sp.vz = _pos_sp_triplet.current.vz;
        break;
      case position_setpoint_s::VELOCITY_FRAME_BODY_NED:
        _local_pos_sp.vx = cosf(_yaw) * _pos_sp_triplet.current.vx -
                           sinf(_yaw) * _pos_sp_triplet.current.vy;
        _local_pos_sp.vy = sinf(_yaw) * _pos_sp_triplet.current.vx +
                           cosf(_yaw) * _pos_sp_triplet.current.vy;
        // We'll keep the yaw-only assumption
        _local_pos_sp.vz = _pos_sp_triplet.current.vz;
        break;
      default:
        // TODO(nathan) maybe warn
        _local_pos_sp.vx = _vel(0);
        _local_pos_sp.vy = _vel(1);
        _local_pos_sp.vz = _vel(2);
        break;
    }
  } else {
    _local_pos_sp.vx = 0.0f;
    _local_pos_sp.vy = 0.0f;
    _local_pos_sp.vz = 0.0f;
  }

  _local_pos_sp.acceleration[0] = _pos_sp_triplet.current.a_x;
  _local_pos_sp.acceleration[1] = _pos_sp_triplet.current.a_y;
  _local_pos_sp.acceleration[2] = _pos_sp_triplet.current.a_z;

  if (_pos_sp_triplet.current.yaw_valid) {
    _local_pos_sp.yaw = _pos_sp_triplet.current.yaw;
  }

  _local_pos_sp.yawspeed = NAN;
}

void AdaptiveMulticopterPositionControl::_fill_setpoint_offboard() {
  if (not _pos_sp_triplet.current.valid) {
    _fill_setpoint_idle();
    // TODO(nathan) warn people here
    return;
  }

  switch (_pos_sp_triplet.current.type) {
    case position_setpoint_s::SETPOINT_TYPE_POSITION:
    case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
    case position_setpoint_s::SETPOINT_TYPE_OFFBOARD:
    case position_setpoint_s::SETPOINT_TYPE_GRASP_TRAJECTORY:
    case position_setpoint_s::SETPOINT_TYPE_GRASP_TRAJECTORY_NO_GE:
      _fill_setpoint_pose_triplet();
      // log_warn("Type: SETPOINT");
      break;
    case position_setpoint_s::SETPOINT_TYPE_LOITER:
      _fill_setpoint_hold();
      // log_warn("Type: HOLD");
      break;
    case position_setpoint_s::SETPOINT_TYPE_TAKEOFF:
      _fill_setpoint_takeoff();
      // log_warn("Type: TAKEOFF");
      break;
    case position_setpoint_s::SETPOINT_TYPE_LAND:
      _fill_setpoint_land();
      // log_warn("Type: LAND");
      break;
    case position_setpoint_s::SETPOINT_TYPE_IDLE:
      _fill_setpoint_idle();
      // log_warn("Type: IDLE");
      break;
    default:
      _fill_setpoint_idle();
      // log_warn("Type: UNKNOWN");
      // TODO(nathan) think about warning people here
      break;
  }
}

void AdaptiveMulticopterPositionControl::_publish_attitude_setpoint(float dt) {
  VehicleState state;
  state.R = _R;
  state.pos = _pos;
  state.vel = _vel;
  // TODO(nathan) this is bad, but tricky otherwise
  state.acc.zero();
  state.dt = dt;
  // TODO(nathan) populate this with acceleration
  std::memcpy(state.motor_pwm_values, _actuator_outputs.output,
              sizeof(state.motor_pwm_values));

  const bool in_takeoff =
      _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
  /*  const bool in_land =*/
  /*_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND;*/
  // const bool not_trajectory = in_takeoff || in_land;
  const bool not_trajectory = in_takeoff;

  _position_controller.adaptive_params.use_adaptive_term =
      _adaptive_term_en.get() && not _is_idle && not not_trajectory;

  _position_controller.run(state, _local_pos_sp, _att_sp);

  if (_is_idle) {
    _att_sp.thrust_body[2] = 0.0f;
  }

  _att_sp.timestamp = hrt_absolute_time();
  _att_sp.adap_x = _position_controller._bar_theta_x(0);
  _att_sp.adap_y = _position_controller._bar_theta_x(1);
  _att_sp.adap_z = _position_controller._bar_theta_x(2);
  _att_sp.is_idle = _is_idle;

  if (!_control_mode.flag_control_manual_enabled) {
      if (not _att_sp_pub) {
        _att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
      }

      orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);
  }
}

void AdaptiveMulticopterPositionControl::run() {
  _initialize_subscriptions();
  _update_from_subscriptions();

  // TODO(nathan) we'll probably keep this, but helper function!!!!
  while (not should_exit()) {
    UorbPoll::Info info = should_run(_local_pos_sub);

    _update_from_subscriptions();  // still check subscriptions no matter what
    _update_pose_information();

    if (not info.have_data) {
      continue;
    }

    perf_begin(_loop_perf);

    _update_parameters(false);
    _cache_pose();

    _is_idle = false;
    if (_control_mode.flag_control_manual_enabled) {
      // TODO(nathan) cache new setpoint here
      // _fill_setpoint_manual(info.time_elapsed);
      _fill_setpoint_idle();
    } else if (_control_mode.flag_control_offboard_enabled) {
      _fill_setpoint_offboard();
    } else {
      _fill_setpoint_idle();
    }

    // publish the setpoint handed to the controller
    _publish_local_position_setpoint();
    // do control and publish setpoint
    _publish_attitude_setpoint(info.time_elapsed);

    _prev_setpoint = _get_setpoint_type();
    perf_end(_loop_perf);
  }

  _cleanup_subscriptions();
}

int AdaptiveMulticopterPositionControl::print_status() {
  //PX4_INFO("Adaptive -> (up: %s en: %s deriv: %s)",
  //         (_weight_update_en.get() ? "yes" : "no"),
  //         (_adaptive_term_en.get() ? "yes" : "no"),
  //         (_weight_derivative_update_en.get() ? "yes" : "no"));
  //PX4_INFO("Ground Effect -> (up: %s en: %s)",
  //         (_ground_effect_update_en.get() ? "yes" : "no"),
  //         (_ground_effect_term_en.get() ? "yes" : "no"));
  show_poll_rate();
  return 0;
}

int AdaptiveMulticopterPositionControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This implements an adaptive multicopter position controller.

### Implementation
TBD

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

int AdaptiveMulticopterPositionControl::task_spawn(int argc, char *argv[]) {
  _task_id = px4_task_spawn_cmd("mc_adapt_pos_control", SCHED_DEFAULT,
                                SCHED_PRIORITY_POSITION_CONTROL, 1900,
                                (px4_main_t)&run_trampoline, (char *const *)argv);

  if (_task_id < 0) {
    _task_id = -1;
    return -errno;
  }

  return 0;
}

AdaptiveMulticopterPositionControl *AdaptiveMulticopterPositionControl::instantiate(
    int argc, char *argv[]) {
  return new AdaptiveMulticopterPositionControl();
}

int AdaptiveMulticopterPositionControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

}  // namespace mc_adaptive_control

int mc_adaptive_pos_control_main(int argc, char *argv[]) {
  return mc_adaptive_control::AdaptiveMulticopterPositionControl::main(argc, argv);
}
