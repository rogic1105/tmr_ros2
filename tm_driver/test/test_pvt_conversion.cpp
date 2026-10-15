#include "tm_driver/tm_pvt_conversion.h"
#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>

struct Duration { int32_t sec; uint32_t nanosec; };
struct TrajectoryPoint {
  Duration time_from_start;
  std::vector<double> positions, velocities;
};
static TrajectoryPoint point(uint64_t ns, double position = 1.0) {
  return {{static_cast<int32_t>(ns / 1000000000), static_cast<uint32_t>(ns % 1000000000)},
          std::vector<double>(6, position), std::vector<double>(6, 0.0)};
}
int main() {
  TmPvtTraj output;
  std::string error;
  const auto convert = [&](const std::vector<TrajectoryPoint> &input) {
    return tm_convert_joint_pvt(output, input, 0.025, error);
  };
  // Actual crash input: two identical points, zero velocities, 8.40997 ms.
  assert(!convert({point(0), point(8409970)}));
  assert(error.find("duration") != std::string::npos && output.points.empty());
  assert(!convert({}));
  assert(!convert({point(0)}));
  assert(!convert({point(100000000)}));
  assert(!convert({point(0), point(24999999)}));
  assert(convert({point(0), point(25000000, 2.0)}));
  assert(output.points.size() == 1 && output.points.back().positions[0] == 2.0);
  assert(std::fabs(output.total_time - 0.025) < 1e-12);
  // Filter short interior points and merge a short terminal segment without
  // altering endpoint or total duration (no invented time stretching).
  assert(convert({point(0), point(10000000), point(50000000), point(60000000, 3.0)}));
  assert(output.points.size() == 1 && output.points[0].positions[0] == 3.0);
  assert(std::fabs(output.points[0].time - 0.060) < 1e-12);
  assert(std::fabs(output.total_time - 0.060) < 1e-12);
  // Existing two intervals retain their timing when the terminal segment merges.
  assert(convert({point(0), point(50000000), point(100000000), point(108409970, 4.0)}));
  assert(output.points.size() == 2);
  assert(std::fabs(output.points[0].time - 0.05) < 1e-12);
  assert(std::fabs(output.points[1].time - 0.058409970) < 1e-12);
  assert(output.points.back().positions[0] == 4.0);
  assert(convert({point(10000000), point(30000000)}));
  assert(output.points.size() == 1 && std::fabs(output.points[0].time - 0.03) < 1e-12);
  assert(convert({point(0), point(100000000, 2.0), point(200000000, 3.0)}));
  assert(output.points.size() == 2);
  assert(std::fabs(output.points[0].time - 0.1) < 1e-12);
  assert(std::fabs(output.points[1].time - 0.1) < 1e-12);
  assert(std::fabs(output.total_time - 0.2) < 1e-12);
  assert(!convert({point(0), point(0)}));
  assert(!convert({point(50000000), point(25000000)}));
  auto malformed = point(100000000);
  malformed.time_from_start.nanosec = 1000000000u;
  assert(!convert({point(0), malformed}));
  malformed = point(100000000);
  malformed.positions.clear();
  assert(!convert({point(0), malformed}));
  malformed = point(100000000);
  malformed.velocities[0] = std::numeric_limits<double>::quiet_NaN();
  assert(!convert({point(0), malformed}));
  assert(output.points.empty() && output.total_time == 0.0);
}
