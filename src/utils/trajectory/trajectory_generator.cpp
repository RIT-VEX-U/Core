#include "core/utils/trajectory/trajectory_generator.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "core/utils/math/spline/spline_path.h"
#include "core/utils/trajectory/trajectory_parameterizer.h"

const Trajectory TrajectoryGenerator::kDoNothingTrajectory(
    std::vector<Trajectory::State>{Trajectory::State()});

namespace {

std::vector<PoseWithCurvature> spline_points_from_hermite(
    const std::vector<HermitePoint>& waypoints,
    double step_ds,
    SplinePath::Order order) {
  std::vector<PoseWithCurvature> out;
  if (waypoints.size() < 2) {
    return out;
  }

  SplinePath spline_path = SplinePath::from_hermite(waypoints, order);
  const double total_length = spline_path.length();
  if (total_length <= 1e-9) {
    return out;
  }

  const double sample_step = std::max(1e-3, step_ds);
  for (double s = 0.0; s < total_length - 1e-6; s += sample_step) {
    const SplineSample sample = spline_path.sample_by_s(s);
    out.push_back({Pose2d(sample.position, sample.heading), sample.curvature});
  }

  const SplineSample end_sample = spline_path.sample_by_s(total_length);
  out.push_back({Pose2d(end_sample.position, end_sample.heading), end_sample.curvature});

  return out;
}

}  // namespace

Trajectory TrajectoryGenerator::generate_trajectory(
    const std::vector<HermitePoint>& waypoints,
    const TrajectoryConfig& config) {
  std::vector<PoseWithCurvature> points = spline_points_from_hermite(waypoints, config.sample_ds().to(units::in), config.spline_order());
  if (points.empty()) {
    config.error_handler()("TrajectoryGenerator: Could not generate spline points.");
    return kDoNothingTrajectory;
  }

  if (config.is_reversed()) {
    const Transform2d flip{Translation2d{}, Rotation2d(180_deg)};
    for (PoseWithCurvature& point : points) {
      point = {point.pose + flip, -point.curvature};
    }
  }

  Trajectory traj = TrajectoryParameterizer::time_parameterize_trajectory(
      points,
      config.constraints(),
      config.start_velocity(),
      config.end_velocity(),
      config.max_velocity(),
      config.max_acceleration(),
      config.is_reversed(),
      config.error_handler());


  std::vector<TrajectoryEvent> evs;
  for (const DistanceEvent& ev : config.events()) {
    evs.push_back({traj.time_from_distance(ev.distance), ev.name});
  }
  traj.set_events(std::move(evs));

  return traj;
}

Trajectory TrajectoryGenerator::generate_trajectory(
    const std::vector<Pose2d>& waypoints,
    const TrajectoryConfig& config,
    double tangent_scale) {
  std::vector<HermitePoint> hermite_points;
  hermite_points.reserve(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    double speed = 0.0;
    if (i == 0) {
      speed = waypoints[i].translation().distance(waypoints[i+1].translation()).to(units::in) *
              tangent_scale;
    } else if (i == waypoints.size() - 1) {
      speed = waypoints[i].translation().distance(waypoints[i-1].translation()).to(units::in) *
              tangent_scale;
    } else {
      double d_next = waypoints[i].translation().distance(waypoints[i+1].translation()).to(units::in);
      double d_prev = waypoints[i].translation().distance(waypoints[i-1].translation()).to(units::in);
      speed = (d_next + d_prev) / 2.0 * tangent_scale;
    }
    
    Pose2d wp = waypoints[i];
    if (config.is_reversed()) {
      const Transform2d flip{Translation2d{}, Rotation2d(180_deg)};
      wp = wp + flip;
    }
    hermite_points.push_back(HermitePoint::from_pose(wp, speed));
  }

  return generate_trajectory(hermite_points, config);
}

Trajectory TrajectoryGenerator::generate_trajectory(
    const Pose2d& current_pose,
    units::Velocity current_velocity,
    const std::vector<Pose2d>& target_waypoints,
    TrajectoryConfig config,
    double tangent_scale) {

  std::vector<Pose2d> full_waypoints;
  full_waypoints.reserve(target_waypoints.size() + 1);
  full_waypoints.push_back(current_pose);
  for (const Pose2d& wp : target_waypoints) {
    full_waypoints.push_back(wp);
  }

  config.set_start_velocity(current_velocity);
  return generate_trajectory(full_waypoints, config, tangent_scale);
}
