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
 * @file mc_pos_control.hpp
 * Adaptive multicopter position controller.
 *
 * Heavily refactored position controller that more closely follows
 * the original geometric control paper.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>

 */
#include <mc_adaptive_control_utils/geometric_controller.hpp>
#include <mc_adaptive_control_utils/px4_helpers.hpp>
#include <mc_adaptive_pos_control/manual_input_handler.hpp>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <matrix/matrix/math.hpp>

extern "C" __EXPORT int mc_adaptive_pos_control_main(int argc, char *argv[]);

enum class SetpointType : int {
  NONE = 0,
  IDLE = 1,
  HOLD = 2,
  TAKEOFF = 3,
  LAND = 4,
  POSE_TRIPLET = 5,
};

namespace mc_adaptive_control {

class AdaptiveMulticopterPositionControl
    : public ModuleBase<AdaptiveMulticopterPositionControl>,
      public ModuleParams,
      public UorbPoll,
      public PeriodicLog {
 public:
  AdaptiveMulticopterPositionControl();

  virtual ~AdaptiveMulticopterPositionControl() = default;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static AdaptiveMulticopterPositionControl *instantiate(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  /** @see ModuleBase::run() */
  void run() override;

  /** @see ModuleBase::print_status() */
  int print_status() override;

 private:
  void _initialize_subscriptions();
  void _update_from_subscriptions();
  void _cleanup_subscriptions();
  void _update_parameters(bool force);

  void _update_pose_information();
  void _cache_pose();
  auto _get_setpoint_type() -> SetpointType;

  void _fill_setpoint_hold();
  void _fill_setpoint_takeoff();
  void _fill_setpoint_land();
  void _fill_setpoint_idle();
  void _fill_setpoint_pose_triplet();
  void _fill_setpoint_offboard();

  void _publish_attitude_setpoint(float dt);

  void _publish_local_position_setpoint();

  GeometricPosController _position_controller;

  perf_counter_t _loop_perf;

  SetpointType _prev_setpoint;

  matrix::Vector3f _pos;
  matrix::Vector3f _vel;
  matrix::Vector3f _acc;
  matrix::Vector3f _cached_pos;
  float _cached_yaw;
  matrix::Dcmf _R;
  float _yaw;

  bool _is_idle;

  int _vehicle_status_sub;
  int _vehicle_land_detected_sub;
  int _vehicle_attitude_sub;
  int _control_mode_sub;
  int _params_sub;
  int _manual_sub;
  int _local_pos_sub;
  int _pos_sp_triplet_sub;
  int _actuator_outputs_sub;

  orb_advert_t _att_sp_pub;
  orb_advert_t _local_pos_sp_pub;

  struct vehicle_status_s _vehicle_status;
  struct vehicle_land_detected_s _vehicle_land_detected;
  struct vehicle_attitude_s _att;
  struct manual_control_setpoint_s _manual;
  struct vehicle_control_mode_s _control_mode;
  struct parameter_update_s _param_update;
  struct vehicle_local_position_s _local_pos;
  struct position_setpoint_triplet_s _pos_sp_triplet;
  struct actuator_outputs_s _actuator_outputs;

  // TODO(nathan) rethink the attitude setpoint struct
  struct vehicle_attitude_setpoint_s _att_sp;
  struct vehicle_local_position_setpoint_s _local_pos_sp;

  ManualInputHandler _manual_input_handler;

  DEFINE_PARAMETERS((ParamBool<px4::params::AMPC_ADAP_EN>)_adaptive_term_en,
                    (ParamInt<px4::params::AMPC_ADOT_MODE>)_adot_mode,
                    (ParamFloat<px4::params::AMPC_KP>)_position_gain,
                    (ParamFloat<px4::params::AMPC_KV>)_velocity_gain,
                    (ParamFloat<px4::params::AMPC_C1>)_c1_gain,
                    (ParamFloat<px4::params::AMPC_GAMMA_X>)_gamma_x_gain,
                    (ParamFloat<px4::params::AMPC_ADAP_B_TH_X>)_B_theta_x,
                    (ParamFloat<px4::params::AMPC_ADAP_LIM>)_adaptive_limit,
                    (ParamBool<px4::params::AMPC_LOG_ENABLE>)_log_enable,
                    (ParamFloat<px4::params::AMC_MASS>)_vehicle_mass,
                    (ParamFloat<px4::params::AMC_JXX>)_vehicle_Jxx,
                    (ParamFloat<px4::params::AMC_JYY>)_vehicle_Jyy,
                    (ParamFloat<px4::params::AMC_JZZ>)_vehicle_Jzz,
                    (ParamFloat<px4::params::MPC_LAND_SPEED>)_land_speed);

};

}  // namespace mc_adaptive_control
