#include <mc_adaptive_pos_control/manual_input_handler.hpp>

#include <mathlib/mathlib.h>

namespace mc_adaptive_control {

typedef struct vehicle_control_mode_s vehicle_control_mode;
typedef struct manual_control_setpoint_s manual_control_setpoint;
typedef struct position_setpoint_triplet_s position_setpoint_triplet;

ManualInputHandler::ManualInputHandler()
    : _pitch_filter(50.0f, 10.0f), _roll_filter(50.0f, 10.0f) {}

void ManualInputHandler::update_filters(float sample_rate, float cutoff_frequency) {
  double actual_rate = math::max(sample_rate, 1.0f);
  double actual_cutoff = math::min(cutoff_frequency, (sample_rate / 2.0f) - 1.0f);
  _pitch_filter.set_cutoff_frequency(actual_rate, actual_cutoff);
  _roll_filter.set_cutoff_frequency(actual_rate, actual_cutoff);
}

auto ManualInputHandler::get_vehicle_setpoint(const vehicle_control_mode& control_mode,
                                              const manual_control_setpoint& input)
    -> position_setpoint_triplet {
  return {};
}

}  // namespace mc_adaptive_control
