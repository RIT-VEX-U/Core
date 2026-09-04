#include "core/utils/trajectory/trajectory_generator.h"

#include <algorithm>
#include <cstdio>
#include <utility>
#include <vector>

#include "core/utils/math/spline/spline_path.h"
#include "core/utils/trajectory/trajectory_parameterizer.h"

const Trajectory TrajectoryGenerator::kDoNothingTrajectory(
    std::vector<Trajectory::State>{Trajectory::State()});
std::function<void(const char*)> TrajectoryGenerator::s_errorFunc;

void TrajectoryGenerator::report_error(const char* error) {
  if (s_errorFunc) {
    s_errorFunc(error);
  } else {
    std::fprintf(stderr, "TrajectoryGenerator error: %s\n", error);
  }
}

namespace {

std::vector<TrajectoryGenerator::PoseWithCurvature> spline_points_from_hermite(
    const std::vector<HermitePoint>& waypoints,
    double step_ds) {
  std::vector<TrajectoryGenerator::PoseWithCurvature> out;
  if (waypoints.size() < 2) {
    return out;
  }

  SplinePath spline_path = SplinePath::from_hermite(waypoints, SplinePath::Order::Quintic);
  const double total_length = spline_path.length();
  if (total_length <= 1e-9) {
    return out;
  }

  const double sample_step = std::max(1e-3, step_ds);
  for (double s = 0.0; s < total_length - 1e-6; s += sample_step) {
    const SplineSample sample = spline_path.sample_by_s(s);
    out.push_back(std::make_pair(Pose2d(sample.position, sample.heading), sample.curvature));
  }

  const SplineSample end_sample = spline_path.sample_by_s(total_length);
  out.push_back(std::make_pair(Pose2d(end_sample.position, end_sample.heading), end_sample.curvature));

  return out;
}

}  // namespace

Trajectory TrajectoryGenerator::generate_trajectory(
    const std::vector<HermitePoint>& waypoints,
    const TrajectoryConfig& config) {
  std::vector<PoseWithCurvature> points = spline_points_from_hermite(waypoints, config.sample_ds().in());
  if (points.empty()) {
    report_error("Could not generate spline points.");
    return kDoNothingTrajectory;
  }

  if (config.is_reversed()) {
    const Transform2d flip{Translation2d{}, from_degrees(180)};
    for (auto& point : points) {
      point = {point.first + flip, -point.second};
    }
  }

  return TrajectoryParameterizer::time_parameterize_trajectory(
      points,
      config.constraints(),
      config.start_velocity(),
      config.end_velocity(),
      config.max_velocity(),
      config.max_acceleration(),
      config.is_reversed());
}

void TrajectoryGenerator::set_error_handler(
    std::function<void(const char*)> func) {
  s_errorFunc = std::move(func);
}
