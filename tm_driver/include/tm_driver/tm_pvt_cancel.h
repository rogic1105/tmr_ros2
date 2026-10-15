#pragma once
#include <atomic>
#include <mutex>
#include <cstdint>
#include <string>

// TM Listen-node transaction IDs permit ASCII letters/digits only. Any other
// byte (including underscore) makes the controller reject the STOP as CPERR04.
inline std::string tm_pvt_stop_id(std::uint64_t clock_ticks, std::uint64_t sequence) {
  return "PvtStop" + std::to_string(clock_ticks) + "S" + std::to_string(sequence);
}

// Serialize the last send decision with stop, including cancel-before-PVT-ACK.
// Reset is legal only when a new, nonoverlapping action is admitted.
class TmPvtCancelGate {
  std::mutex mutex_;
  std::atomic<bool> canceled_{false};
  std::atomic<bool> failed_stop_{false};
public:
  bool reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_stop_) return false;  // Only constructing a new driver clears this.
    canceled_ = false;
    return true;
  }
  bool failed_stop() const { return failed_stop_.load(); }
  bool canceled() const { return canceled_.load(); }
  template<class Send> bool send_if_active(Send send) {
    std::lock_guard<std::mutex> lock(mutex_);
    return !failed_stop_ && !canceled_ && send();
  }
  template<class Stop> bool cancel_and_confirm(Stop stop) {
    std::lock_guard<std::mutex> lock(mutex_);
    canceled_ = true;  // Failed/missing ACK must remain latched as well.
    try {
      const bool confirmed = stop();
      if (!confirmed) failed_stop_ = true;
      return confirmed && !failed_stop_;
    } catch (...) {
      failed_stop_ = true;
      throw;
    }
  }
};
