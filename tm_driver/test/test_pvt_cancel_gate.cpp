#include "tm_driver/tm_pvt_cancel.h"
#include <cassert>
#include <future>
#include <thread>
#include <vector>
#include <string>
#include <limits>

int main() {
  // Test the real wire-ID generator, not a substitute test-only spelling.
  const auto valid_id = [](const std::string &id) {
    for (char c : id)
      if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')))
        return false;
    return !id.empty();
  };
  assert(!valid_id("PvtStop123_1"));  // Reproduces the rejected physical STOP ID.
  assert(valid_id(tm_pvt_stop_id(123, 1)));
  assert(valid_id(tm_pvt_stop_id(std::numeric_limits<std::uint64_t>::max(),
                                 std::numeric_limits<std::uint64_t>::max())));
  assert(tm_pvt_stop_id(123, 1) != tm_pvt_stop_id(123, 2));
  assert(tm_pvt_stop_id(12, 31) != tm_pvt_stop_id(123, 1));

  TmPvtCancelGate gate;
  std::vector<std::string> wire;
  // Cancellation before controller ACK: STOP must be sent even though no
  // trajectory ACK/completion marker exists. No further QueueTag/PVT follows.
  assert(gate.send_if_active([&] { wire.push_back("PVT"); return true; }));
  assert(gate.cancel_and_confirm([&] { wire.push_back("STOP_ACK"); return true; }));
  assert(!gate.send_if_active([&] { wire.push_back("QueueTag"); return true; }));
  assert((wire == std::vector<std::string>{"PVT", "STOP_ACK"}));
  // Cancel before execution thread starts prevents a late PVT script entirely.
  assert(gate.reset()); wire.clear();
  assert(gate.cancel_and_confirm([&] { wire.push_back("STOP_ACK"); return true; }));
  assert(!gate.send_if_active([&] { wire.push_back("PVT"); return true; }));
  assert(wire.size() == 1);
  // Missing/rejected ACK is permanent for this driver instance. Neither a
  // new-goal reset nor a later successful STOP can erase the uncertainty.
  TmPvtCancelGate failed;
  assert(!failed.cancel_and_confirm([] { return false; }));
  assert(failed.canceled() && failed.failed_stop());
  assert(!failed.reset());
  assert(!failed.send_if_active([] { return true; }));
  assert(!failed.cancel_and_confirm([] { return true; }));
  assert(!failed.reset());
  assert(!failed.send_if_active([] { return true; }));
  // A confirmed stop on a healthy instance permits the next admitted goal.
  assert(gate.reset());
  assert(gate.send_if_active([] { return true; }));
  // Concurrent cancel waits for an in-flight send: STOP follows PVT, never vice versa.
  assert(gate.reset()); wire.clear();
  std::promise<void> entered, release;
  auto released = release.get_future();
  std::thread sender([&] { assert(gate.send_if_active([&] {
    wire.push_back("PVT"); entered.set_value(); released.wait(); return true;
  })); });
  entered.get_future().wait();
  auto stop = std::async(std::launch::async, [&] {
    return gate.cancel_and_confirm([&] { wire.push_back("STOP_ACK"); return true; });
  });
  release.set_value(); sender.join(); assert(stop.get());
  assert((wire == std::vector<std::string>{"PVT", "STOP_ACK"}));
  assert(!gate.send_if_active([] { return true; }));
}
