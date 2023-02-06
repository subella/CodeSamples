#pragma once
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

namespace mc_adaptive_control {

class ManualInputHandler {
 public:
  ManualInputHandler();
  virtual ~ManualInputHandler() = default;

  void update_filters(float sample_rate, float cutoff_frequency);

  auto get_vehicle_setpoint(const struct vehicle_control_mode_s& control_mode,
                            const struct manual_control_setpoint_s& input)
      -> struct position_setpoint_triplet_s;

 private:
  math::LowPassFilter2p _pitch_filter;
  math::LowPassFilter2p _roll_filter;

  // matrix::Vector2f _stick_input_xy_prev;
  // _stick_input_xy_prev.zero();
};
}  // namespace mc_adaptive_control
