/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_att_adaptive_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 * @author Nathan Hughes    <nathan.h.hughes@gmail.com>
 *
 */

#include <mc_adaptive_att_control/mc_adaptive_att_control.hpp>
#include <mc_adaptive_control_utils/px4_helpers.hpp>

#include <circuit_breaker/circuit_breaker.h>
#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

namespace mc_adaptive_control {

AdaptiveMulticopterAttitudeControl::AdaptiveMulticopterAttitudeControl()
    : ModuleParams(nullptr),
      UorbPoll(),
      _angular_velocity_filters{{get_average_rate(), DEFAULT_FILTER_CUTOFF_HZ},
                                {get_average_rate(), DEFAULT_FILTER_CUTOFF_HZ},
                                {get_average_rate(), DEFAULT_FILTER_CUTOFF_HZ}},
      _loop_perf(perf_alloc(PC_ELAPSED, "mc_adapt_att_control")) {
  for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
    _sensor_gyro_sub[i] = -1;
  }

  _v_att.q[0] = 1.0f;
  _v_att_sp.q_d[0] = 1.0f;

  _wrench_sp.zero();
  _att_control.zero();
  _prev_angular_velocity.zero();

  for (unsigned i = 0; i < 3; i++) {
    //_sensor_correction.gyro_scale_0[i] = 1.0f;
    //_sensor_correction.gyro_scale_1[i] = 1.0f;
    //_sensor_correction.gyro_scale_2[i] = 1.0f;
    _angular_velocity_filters[i] =
        math::LowPassFilter2p(get_average_rate(), DEFAULT_FILTER_CUTOFF_HZ);
  }

  parameters_updated();
}

void AdaptiveMulticopterAttitudeControl::parameters_updated() {
  // TODO(nathan) fix this match pos control
  _actuators_0_circuit_breaker_enabled =
      circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

  _attitude_controller.gains.kr = _rotation_gain.get();
  _attitude_controller.gains.komega = _omega_gain.get();
  _attitude_controller.gains.c2 = _c2_gain.get();
  _attitude_controller.gains.gamma_r = _gamma_r_gain.get();
  _attitude_controller.properties.m = _vehicle_mass.get();
  _attitude_controller.properties.J(0, 0) = _vehicle_Jxx.get();
  _attitude_controller.properties.J(1, 1) = _vehicle_Jyy.get();
  _attitude_controller.properties.J(2, 2) = _vehicle_Jzz.get();
  //_attitude_controller.adaptive_params.limit = _adaptive_delta_limit.get();

  _board_rotation = get_rot_matrix((enum Rotation)_board_rotation_param.get());
  Dcmf board_rotation_offset(Eulerf(M_DEG_TO_RAD_F * _board_offset_x.get(),
                                    M_DEG_TO_RAD_F * _board_offset_y.get(),
                                    M_DEG_TO_RAD_F * _board_offset_z.get()));
  _board_rotation = board_rotation_offset * _board_rotation;

  const float curr_cutoff = _angular_velocity_filters[0].get_cutoff_freq();
  const float new_cutoff = _angular_velocity_filter_cutoff_hz.get();
  if (fabsf(curr_cutoff - new_cutoff) < 0.01f) {
    return;  // this is safe because this comes last!!!
  }

  for (size_t i = 0; i < 3; ++i) {
    // there's likely some weird behavior when the loop is just initializing, but it
    // should be fine by the time that we actually run something
    _angular_velocity_filters[i].set_cutoff_frequency(get_average_rate(), new_cutoff);
    _angular_velocity_filters[i].reset(_prev_angular_velocity(i));
  }
}

void AdaptiveMulticopterAttitudeControl::_update_from_subscriptions() {
  utils::check_and_fill(ORB_ID(vehicle_control_mode), _v_control_mode_sub,
                        &_v_control_mode);
  utils::check_and_fill(ORB_ID(manual_control_setpoint), _manual_control_sp_sub,
                        &_manual_control_sp);
  utils::check_and_fill(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
  utils::check_and_fill(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
  utils::check_and_fill(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
  utils::check_and_fill(ORB_ID(sensor_correction), _sensor_correction_sub,
                        &_sensor_correction);
  utils::check_and_fill(ORB_ID(estimator_sensor_bias), _sensor_bias_sub, &_sensor_bias);
  utils::check_and_fill(ORB_ID(sensor_selection), _sensor_selection_sub, &_sensor_selection);

  if (utils::check_and_fill(ORB_ID(parameter_update), _params_sub,
                            &_parameter_update)) {
    updateParams();
    parameters_updated();
  }

  _selected_gyro = _sensor_selection.gyro_device_id;
  //if (_sensor_correction.selected_gyro_instance < _gyro_count) {
  //  _selected_gyro = _sensor_correction.selected_gyro_instance;
  //}
}

auto AdaptiveMulticopterAttitudeControl::_get_corrected_angular_velocity() -> Vector3f {
  Vector3f rates;
  
  if (_selected_gyro == 0) {
    rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]);
    rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]);
    rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]);
  } else if (_selected_gyro == 1) {
    rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]);
    rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]);
    rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]);
  } else if (_selected_gyro == 2) {
    rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]);
    rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]);
    rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]);
  } else {
    rates(0) = _sensor_gyro.x;
    rates(1) = _sensor_gyro.y;
    rates(2) = _sensor_gyro.z;
  }

  rates = _board_rotation * rates;

  // TODO(nathan) WTF?
  rates(0) -= _sensor_bias.gyro_bias[0];
  rates(1) -= _sensor_bias.gyro_bias[1];
  rates(2) -= _sensor_bias.gyro_bias[2];

  Vector3f rates_filtered;
  rates_filtered(0) = _angular_velocity_filters[0].apply(rates(0));
  rates_filtered(1) = _angular_velocity_filters[1].apply(rates(1));
  rates_filtered(2) = _angular_velocity_filters[2].apply(rates(2));

  // TODO(nathan) this is to handle a safe reset after a
  // parameter update, but could maybe be rethought
  _prev_angular_velocity = rates;
  _angular_velocity_filtered = rates_filtered;

  if (_angular_velocity_filter_en.get()) {
    return rates_filtered;
  }

  return rates;
}

float AdaptiveMulticopterAttitudeControl::throttle_curve(float throttle_stick_input) {
    const float throttle_min = _param_mpc_manthr_min.get();
    switch (_param_mpc_thr_curve.get()) {
    case 1: // no rescaling
        return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);
    default: // rescale to hover throttle at 0.5 stick
        return math::gradual3(throttle_stick_input, 0.f, .5f, 1.f, throttle_min, _param_mpc_thr_hover.get(), _param_mpc_thr_max.get());
    }
}

void AdaptiveMulticopterAttitudeControl::control_attitude(float dt) {
  Quatf q(_v_att.q);
  Dcmf R(q.normalized());
  Vector3f rates = _get_corrected_angular_velocity();

  VehicleState state;
  state.R = R;
  state.omega = rates;
  state.dt = dt;

  if (_v_control_mode.flag_control_manual_enabled) {
    vehicle_attitude_setpoint_s attitude_setpoint{};
    //const float yaw = Eulerf(q).psi();
    if (_manual_control_sp.z > 0.05f || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {
        const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
        attitude_setpoint.yaw_sp_move_rate = _manual_control_sp.r * yaw_rate;
        _man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
    }
    _man_x_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
    _man_y_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
    _man_x_input_filter.update(_manual_control_sp.x * _man_tilt_max);
    _man_y_input_filter.update(_manual_control_sp.y * _man_tilt_max);
    const float x = _man_x_input_filter.getState();
    const float y = _man_y_input_filter.getState();
    Vector2f v = Vector2f(y, -x);
    float v_norm = v.norm();
    if (v_norm > _man_tilt_max) {
        v *= _man_tilt_max / v_norm;
    }
    Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
    Eulerf euler_sp = q_sp_rpy;
    attitude_setpoint.roll_body = euler_sp(0);
    attitude_setpoint.pitch_body = euler_sp(1);
    attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);
    Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
    q_sp.copyTo(attitude_setpoint.q_d);
    attitude_setpoint.thrust_body[2] = throttle_curve(_manual_control_sp.z);
    attitude_setpoint.timestamp = hrt_absolute_time();

    _attitude_controller.adaptive_params.run_update = true;
    _attitude_controller.adaptive_params.use_adaptive_term = false;
    _attitude_controller.run(state, attitude_setpoint, _wrench_sp);
    //if (_v_att_sp_pub == nullptr) {
    //   _v_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &attitude_setpoint);
    //}
    //orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &attitude_setpoint);
  } else {
    _attitude_controller.adaptive_params.use_adaptive_term =
        _adaptive_term_en.get() && not _v_att_sp.is_idle;

    _attitude_controller.run(state, _v_att_sp, _wrench_sp);
  }
}

void AdaptiveMulticopterAttitudeControl::_initialize_subscriptions() {
  _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
  _v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
  // TODO(nathan) need another vehicle_rates_setpoint sub maybe
  _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
  _params_sub = orb_subscribe(ORB_ID(parameter_update));
  _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
  _battery_status_sub = orb_subscribe(ORB_ID(battery_status));

  _gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

  if (_gyro_count == 0) {
    _gyro_count = 1;
  }

  for (unsigned s = 0; s < _gyro_count; s++) {
    _sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
  }

  _sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

  // force an update for a topic that is published at a low rate
  if (_sensor_correction_sub > 0) {
    orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
  }

  _sensor_bias_sub = orb_subscribe(ORB_ID(estimator_sensor_bias));
}

void AdaptiveMulticopterAttitudeControl::_cleanup_subscriptions() {
  orb_unsubscribe(_v_att_sub);
  orb_unsubscribe(_v_att_sp_sub);
  orb_unsubscribe(_v_control_mode_sub);
  orb_unsubscribe(_params_sub);
  orb_unsubscribe(_manual_control_sp_sub);
  orb_unsubscribe(_battery_status_sub);

  for (unsigned s = 0; s < _gyro_count; s++) {
    orb_unsubscribe(_sensor_gyro_sub[s]);
  }

  orb_unsubscribe(_sensor_correction_sub);
  orb_unsubscribe(_sensor_bias_sub);
}

void AdaptiveMulticopterAttitudeControl::_publish_vehicle_rate_setpoint(float dt) {
  _v_rates_sp.roll = _wrench_sp(0);
  _v_rates_sp.pitch = _wrench_sp(1);
  _v_rates_sp.yaw = _wrench_sp(2);
  _v_rates_sp.thrust_body[2] = _wrench_sp(3);
  _v_rates_sp.adap_x = _attitude_controller._bar_theta_r(0);
  _v_rates_sp.adap_y = _attitude_controller._bar_theta_r(1);
  _v_rates_sp.adap_z = _attitude_controller._bar_theta_r(2);
  // this is safe, as this is called after the new angular velocity is cached
  if (_angular_velocity_filter_en.get()) {
    _v_rates_sp.wx = _angular_velocity_filtered(0);
    _v_rates_sp.wy = _angular_velocity_filtered(1);
    _v_rates_sp.wz = _angular_velocity_filtered(2);
  } else {
    _v_rates_sp.wx = _prev_angular_velocity(0);
    _v_rates_sp.wy = _prev_angular_velocity(1);
    _v_rates_sp.wz = _prev_angular_velocity(2);
  }

  _v_rates_sp.timestamp = hrt_absolute_time();

  if (_v_rates_sp_pub == nullptr) {
    _v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
  }
  orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);
}

//void AdaptiveMulticopterAttitudeControl::_publish_controller_status() {
//  rate_ctrl_status_s rate_ctrl_status;
//  rate_ctrl_status.timestamp = hrt_absolute_time();
//  rate_ctrl_status.rollspeed = _wrench_sp(0);
//  rate_ctrl_status.pitchspeed = _wrench_sp(1);
//  rate_ctrl_status.yawspeed = _wrench_sp(2);
//  rate_ctrl_status.rollspeed_integ = NAN;
//  rate_ctrl_status.pitchspeed_integ = NAN;
//  rate_ctrl_status.yawspeed_integ = NAN;
//
//  int instance;
//  orb_publish_auto(ORB_ID(rate_ctrl_status), &_controller_status_pub, &rate_ctrl_status,
//                   &instance, ORB_PRIO_DEFAULT);
//}

void AdaptiveMulticopterAttitudeControl::_publish_actuators_msg() {
  if (_v_control_mode.flag_control_termination_enabled) {
    _actuators.control[0] = 0.0f;
    _actuators.control[1] = 0.0f;
    _actuators.control[2] = 0.0f;
    _actuators.control[3] = 0.0f;
  } else {
    // rescale thrust to get more control authority
    // TODO(nathan) make this affine
    float thrust = _wrench_sp(3) * _thrust_scale.get();
    _actuators.control[0] = PX4_ISFINITE(_wrench_sp(0)) ? _wrench_sp(0) : 0.0f;
    _actuators.control[1] = PX4_ISFINITE(_wrench_sp(1)) ? _wrench_sp(1) : 0.0f;
    _actuators.control[2] = PX4_ISFINITE(_wrench_sp(2)) ? _wrench_sp(2) : 0.0f;
    _actuators.control[3] = PX4_ISFINITE(thrust) ? thrust : 0.0f;
    //_actuators.control[7] = _v_att_sp.landing_gear;

    // TODO(nathan) should combine this with other lines
    _actuators.control[0] *= _xy_moment_scale.get();
    _actuators.control[1] *= _xy_moment_scale.get();
    _actuators.control[2] *= _z_moment_scale.get();

    // TODO(nathan) check if this is valid
    if (_bat_scale_en.get() && _battery_status.scale > 0.0f) {
      for (int i = 0; i < 4; i++) {
        _actuators.control[i] *= _battery_status.scale;
      }
    }
  }

  _actuators.timestamp = hrt_absolute_time();
  _actuators.timestamp_sample = _sensor_gyro.timestamp;

  if (!_actuators_0_circuit_breaker_enabled) {
    if (_actuators_0_pub == nullptr) {
      _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
    }
    orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
  }
}

void AdaptiveMulticopterAttitudeControl::run() {
  _initialize_subscriptions();

  const hrt_abstime task_start = hrt_absolute_time();
  _last_run = task_start;

  while (not should_exit()) {
    UorbPoll::Info info = should_run(_sensor_gyro_sub[_selected_gyro]);
    if (not info.have_data) {
      continue;
    }

    perf_begin(_loop_perf);

    orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);
    _update_from_subscriptions();

    if (_v_control_mode.flag_control_attitude_enabled) {
      control_attitude(info.time_elapsed);
      _publish_vehicle_rate_setpoint(info.time_elapsed);
    }

    // TODO(nathan) re-examine this logic
    if (_v_control_mode.flag_control_termination_enabled) {
      _wrench_sp.zero();
      _att_control.zero();
    }

    _publish_actuators_msg();
    //_publish_controller_status();

    // TODO(nathan) for our purposes, computing loop rate while disarmed is fine
    // It's probably not super safe in the general case
    if (!_v_control_mode.flag_armed) {
      const float cutoff_hz = _angular_velocity_filter_cutoff_hz.get();

      for (size_t i = 0; i < 3; ++i) {
        // as before, there's probably some weird behavior when resetting
        _angular_velocity_filters[i].set_cutoff_frequency(get_average_rate(),
                                                          cutoff_hz);
        // TODO(nathan) think about whether resetting is safe
        //_angular_velocity_filters[i].reset(_prev_angular_velocity);
      }
    }

    perf_end(_loop_perf);
  }

  _cleanup_subscriptions();
}

int AdaptiveMulticopterAttitudeControl::print_status() {
  //PX4_INFO("Adaptive -> (up: %s en: %s)", (_weight_update_en.get() ? "yes" : "no"),
  //         (_adaptive_term_en.get() ? "yes" : "no"));
  show_poll_rate();
  return 0;
}

int AdaptiveMulticopterAttitudeControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

int AdaptiveMulticopterAttitudeControl::task_spawn(int argc, char *argv[]) {
  _task_id = px4_task_spawn_cmd("mc_adapt_att_control", SCHED_DEFAULT,
                                SCHED_PRIORITY_ATTITUDE_CONTROL, 1700,
                                (px4_main_t)&run_trampoline, (char *const *)argv);

  if (_task_id < 0) {
    _task_id = -1;
    return -errno;
  }

  return 0;
}

AdaptiveMulticopterAttitudeControl *AdaptiveMulticopterAttitudeControl::instantiate(
    int argc, char *argv[]) {
  return new AdaptiveMulticopterAttitudeControl();
}

int AdaptiveMulticopterAttitudeControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

}  // namespace mc_adaptive_control

int mc_adaptive_att_control_main(int argc, char *argv[]) {
  return mc_adaptive_control::AdaptiveMulticopterAttitudeControl::main(argc, argv);
}
