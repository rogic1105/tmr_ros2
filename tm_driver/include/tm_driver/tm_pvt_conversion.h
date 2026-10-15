#pragma once

#include "tm_command.h"
#include <cmath>
#include <string>

// Converts ROS-like trajectory points without ROS runtime dependencies. Short
// interior intervals may merge, but total duration and endpoint remain unchanged.
template <typename Points>
bool tm_convert_joint_pvt(TmPvtTraj &output, const Points &input,
                          double minimum_interval, std::string &error)
{
  output.mode = TmPvtMode::Joint;
  output.points.clear();
  output.total_time = 0.0;
  error.clear();
  const auto reject = [&](const char *reason) {
    output.points.clear();
    output.total_time = 0.0;
    error = reason;
    return false;
  };
  if (!std::isfinite(minimum_interval) || minimum_interval <= 0.0)
    return reject("PVT minimum interval must be finite and positive");
  if (input.size() < 2)
    return reject("PVT trajectory requires at least two points");
  const auto seconds = [](const auto &duration) {
    return static_cast<double>(duration.sec) + 1e-9 * duration.nanosec;
  };
  double previous_time = -1.0;
  for (const auto &point : input) {
    if (point.time_from_start.nanosec >= 1000000000u)
      return reject("PVT duration nanoseconds must be below one second");
    const double time = seconds(point.time_from_start);
    if (!std::isfinite(time) || time < 0.0 || time <= previous_time)
      return reject("PVT timestamps must be nonnegative and strictly increasing");
    if (point.positions.size() != 6 || point.velocities.size() != 6)
      return reject("PVT points require six positions and velocities");
    for (double value : point.positions)
      if (!std::isfinite(value)) return reject("PVT positions must be finite");
    for (double value : point.velocities)
      if (!std::isfinite(value)) return reject("PVT velocities must be finite");
    previous_time = time;
  }
  if (previous_time < minimum_interval)
    return reject("PVT trajectory duration is below the 25 ms driver minimum");

  double retained_time = 0.0;
  for (size_t index = 0; index < input.size(); ++index) {
    const auto &source = input[index];
    const double time = seconds(source.time_from_start);
    const double interval = time - retained_time;
    if (interval == 0.0) continue;  // Initial current-position anchor.
    TmPvtPoint point{interval, source.positions, source.velocities};
    if (interval >= minimum_interval) {
      output.points.push_back(point);
      retained_time = time;
    } else if (index + 1 == input.size()) {
      if (output.points.empty())
        return reject("PVT conversion produced no supported intervals");
      point.time += output.points.back().time;
      output.points.back() = point;
    }
  }
  if (output.points.empty()) return reject("PVT conversion produced no intervals");
  output.total_time = previous_time;
  return true;
}
