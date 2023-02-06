/**
 * @file px4_helpers.hpp
 * Various utility functions that PX4 should really have
 *
 * @author Nathan Hughes    <nathan.h.hughes@gmail.com>
 *
 */
#pragma once
#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <limits.h>
#include <math.h>

namespace mc_adaptive_control {

namespace utils {

template <typename Msg>
auto check_and_fill(orb_id_t msg_id, int subscriber_fd, Msg* to_fill) -> bool {
  bool updated;
  orb_check(subscriber_fd, &updated);

  if (updated) {
    orb_copy(msg_id, subscriber_fd, to_fill);
  }

  return updated;
}

}  // namespace utils

class UorbPoll {
 public:
  struct Info {
    float time_elapsed;
    bool have_data;
  };

  UorbPoll(int timeout = 100, float min_dt = 0.002f, float max_dt = 0.02f,
           int error_sleep_time_us = 100000, float discount_factor = 0.9);

  virtual ~UorbPoll() = default;

  auto get_average_rate() -> float;

  auto should_run(int uorb_fd) -> Info;

  void show_poll_rate() const;

 protected:
  px4_pollfd_struct_t _poll_fds;
  hrt_abstime _last_run;
  int _timeout;
  float _min_dt;
  float _max_dt;
  int _error_sleep_time_us;
  float _average_dt;
  float _average_clipped_dt;
  float _discount_factor;
  float _sample_weight;
  bool _started;
};

class PeriodicLog {
 public:
  PeriodicLog(unsigned int min_period = 200000);

  virtual ~PeriodicLog() = default;

  void set_active(bool active);

  //void log_warn(const char* string);

 protected:
  unsigned int _min_period;
  hrt_abstime _last_time;
  bool _active;
  orb_advert_t _mavlink_log_pub;
};

}  // namespace mc_adaptive_control
