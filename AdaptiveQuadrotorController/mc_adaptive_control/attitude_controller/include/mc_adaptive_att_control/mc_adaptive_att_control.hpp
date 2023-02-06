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

#include <mc_adaptive_control_utils/geometric_controller.hpp>
#include <mc_adaptive_control_utils/px4_helpers.hpp>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <matrix/matrix/math.hpp>

#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>
#include <lib/mixer/MixerBase/Mixer.hpp> // Airmode

#define MAX_GYRO_COUNT 3

extern "C" __EXPORT int mc_adaptive_att_control_main(int argc, char *argv[]);

namespace mc_adaptive_control {

class AdaptiveMulticopterAttitudeControl
    : public ModuleBase<AdaptiveMulticopterAttitudeControl>,
      public ModuleParams,
      UorbPoll {
 public:
  static constexpr float DEFAULT_FILTER_CUTOFF_HZ = 100.0f;

  AdaptiveMulticopterAttitudeControl();

  virtual ~AdaptiveMulticopterAttitudeControl() = default;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static AdaptiveMulticopterAttitudeControl *instantiate(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  /** @see ModuleBase */
  int print_status() override;

  /** @see ModuleBase::run() */
  void run() override;

 private:
  void _initialize_subscriptions();
  void _update_from_subscriptions();
  void _cleanup_subscriptions();
  void _publish_vehicle_rate_setpoint(float dt);
  void _publish_controller_status();
  void _publish_actuators_msg();
  auto _get_corrected_angular_velocity() -> matrix::Vector3f;

  // TODO(nathan) clean this up to match pos control
  void parameters_updated();
  float throttle_curve(float throttle_stick_input);
  void control_attitude(float dt);

  GeometricAttController _attitude_controller;

  int _v_att_sub{-1};
  int _v_att_sp_sub{-1};
  int _v_control_mode_sub{-1};
  int _params_sub{-1};
  int _manual_control_sp_sub{-1};
  int _battery_status_sub{-1};
  int _sensor_gyro_sub[MAX_GYRO_COUNT];
  int _sensor_correction_sub{-1};
  int _sensor_bias_sub{-1};
  int _sensor_selection_sub{-1};


  unsigned _gyro_count{1};
  int _selected_gyro{0};

  orb_advert_t _v_rates_sp_pub{nullptr};
  orb_advert_t _actuators_0_pub{nullptr};
  orb_advert_t _controller_status_pub{nullptr};
  orb_advert_t _v_att_sp_pub{nullptr};

  bool _actuators_0_circuit_breaker_enabled{false};

  struct vehicle_attitude_s _v_att {};
  struct vehicle_attitude_setpoint_s _v_att_sp {};
  struct vehicle_rates_setpoint_s _v_rates_sp {};
  struct manual_control_setpoint_s _manual_control_sp {};
  struct vehicle_control_mode_s _v_control_mode {};
  struct actuator_controls_s _actuators {};
  struct battery_status_s _battery_status {};
  struct sensor_gyro_s _sensor_gyro {};
  struct sensor_correction_s _sensor_correction {};
  //struct sensor_bias_s _sensor_bias {};
  struct sensor_selection_s _sensor_selection {};
  struct estimator_sensor_bias_s _sensor_bias {};
  struct parameter_update_s _parameter_update {};

  matrix::Vector4f _wrench_sp;
  matrix::Vector3f _att_control;

  matrix::Dcmf _board_rotation;

  math::LowPassFilter2p _angular_velocity_filters[3];
  matrix::Vector3f _prev_angular_velocity;
  matrix::Vector3f _angular_velocity_filtered;

  perf_counter_t _loop_perf;

  float _man_yaw_sp{0.f};
  float _man_tilt_max;
  AlphaFilter<float> _man_x_input_filter;
  AlphaFilter<float> _man_y_input_filter;

  DEFINE_PARAMETERS((ParamBool<px4::params::AMC_ADAP_EN>)_adaptive_term_en,
                    (ParamBool<px4::params::AMC_FILT_ANGV>)_angular_velocity_filter_en,
                    (ParamFloat<px4::params::AMC_FILT_FREQ>)_angular_velocity_filter_cutoff_hz,
                    (ParamFloat<px4::params::AMC_KROT>)_rotation_gain,
                    (ParamFloat<px4::params::AMC_KOMEGA>)_omega_gain,
                    (ParamFloat<px4::params::AMC_C2>)_c2_gain,
                    (ParamFloat<px4::params::AMC_GAMMA_R>)_gamma_r_gain,
                    (ParamFloat<px4::params::AMC_ADAP_LIM>)_adaptive_delta_limit,
                    (ParamFloat<px4::params::AMC_XY_MOM_SCL>)_xy_moment_scale,
                    (ParamFloat<px4::params::AMC_Z_MOM_SCL>)_z_moment_scale,
                    (ParamFloat<px4::params::AMC_THRUST_SCL>)_thrust_scale,
                    (ParamFloat<px4::params::AMC_MASS>)_vehicle_mass,
                    (ParamFloat<px4::params::AMC_JXX>)_vehicle_Jxx,
                    (ParamFloat<px4::params::AMC_JYY>)_vehicle_Jyy,
                    (ParamFloat<px4::params::AMC_JZZ>)_vehicle_Jzz,

                    (ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,
                    (ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,
                    (ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,
                    (ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
                    (ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
                    (ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,
                    (ParamFloat<px4::params::MC_MAN_TILT_TAU>) _param_mc_man_tilt_tau,
                    (ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,

                    (ParamBool<px4::params::MC_BAT_SCALE_EN>)_bat_scale_en,
                    (ParamInt<px4::params::SENS_BOARD_ROT>)_board_rotation_param,
                    (ParamFloat<px4::params::SENS_BOARD_X_OFF>)_board_offset_x,
                    (ParamFloat<px4::params::SENS_BOARD_Y_OFF>)_board_offset_y,
                    (ParamFloat<px4::params::SENS_BOARD_Z_OFF>)_board_offset_z)
};

}  // namespace mc_adaptive_control
