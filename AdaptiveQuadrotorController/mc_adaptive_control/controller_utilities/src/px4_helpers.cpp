/**
 * @file px4_helpers.cpp
 * Various utility functions that PX4 should really have
 *
 * @author Nathan Hughes    <nathan.h.hughes@gmail.com>
 *
 */
#include <mc_adaptive_control_utils/px4_helpers.hpp>

#include <px4_log.h>
#include <cerrno>

namespace mc_adaptive_control {

UorbPoll::UorbPoll(int timeout, float min_dt, float max_dt, int error_sleep_time_us,
                   float discount_factor)
    : _timeout(timeout),
      _min_dt(min_dt),
      _max_dt(max_dt),
      _error_sleep_time_us(error_sleep_time_us),
      _average_dt(0.0f),
      _average_clipped_dt(0.0f),
      _discount_factor(discount_factor),
      _sample_weight(1.0f - discount_factor) {
  _poll_fds = {};
  _poll_fds.events = POLLIN;
}

auto UorbPoll::get_average_rate() -> float {
  if (fabsf(_average_dt) < 1.0e-6f) {
    return 1.0f / _min_dt;
  }
  return 1.0f / _average_dt;
}

auto UorbPoll::should_run(int uorb_fd) -> UorbPoll::Info {
  _poll_fds.fd = uorb_fd;

  int ret = px4_poll(&_poll_fds, 1, _timeout);
  if (ret < 0) {
    // let the user know something went wrong
    PX4_ERR("poll error %d, %d", ret, errno);
    usleep(_error_sleep_time_us);
  }

  if (ret <= 0) {
    return {.time_elapsed = NAN, .have_data = false};
  }

  const hrt_abstime now = hrt_absolute_time();
  float dt = (now - _last_run) * 1.0e-6f;
  _last_run = now;

  if (not _started) {
    _average_dt = dt;
  } else {
    _average_dt = _sample_weight * dt + _discount_factor * _average_dt;
  }

  // clip dt to prevent any math errors downstream
  if (dt < _min_dt) {
    dt = _min_dt;
  } else if (dt > _max_dt) {
    dt = _max_dt;
  }

  if (not _started) {
    _average_clipped_dt = dt;
  } else {
    _average_clipped_dt = _sample_weight * dt + _discount_factor * _average_clipped_dt;
  }

  if (not _started) {
    _started = true;
  }

  return {.time_elapsed = dt, .have_data = true};
}

void UorbPoll::show_poll_rate() const {
  PX4_INFO("Rate: %6.2lf Adjusted Rate: %6.2lf",
           static_cast<double>(1.0f / _average_dt),
           static_cast<double>(1.0f / _average_clipped_dt));
}

PeriodicLog::PeriodicLog(unsigned int min_period)
    : _min_period(min_period),
      _last_time(0),
      _active(false),
      _mavlink_log_pub(nullptr) {}

void PeriodicLog::set_active(bool active) { _active = active; }

//void PeriodicLog::log_warn(const char* string) {
//  if (not _active) {
//    return;
//  }
//
//  hrt_abstime now = hrt_absolute_time();
//  if (now - _last_time > _min_period) {
//    mavlink_log_info(&_mavlink_log_pub, string);
//    _last_time = now;
//  }
//}
}  // namespace mc_adaptive_control
