#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/vector2d.h"

/**
 * Construct a new Odometry Base object
 *
 * @param is_async True to run constantly in the background, false to call update() manually
 */
OdometryBase::OdometryBase(bool is_async) : current_pos(zero_pos) {
  if (is_async) {
    handle = new vex::task(background_task, (void *)this);
  }
}

/**
 * Function that runs in the background task. This function pointer is passed
 * to the vex::task constructor.
 *
 * @param ptr Pointer to OdometryBase object
 * @return Required integer return code. Unused.
 */
int OdometryBase::background_task(void *ptr) {
  OdometryBase &obj = *((OdometryBase *)ptr);
  vexDelay(1000);
  while (!obj.end_task) {
    obj.mut.lock();
    obj.update();
    obj.mut.unlock();
    vexDelay(5);
  }

  return 0;
}

/**
 * End the background task. Cannot be restarted.
 * If the user wants to end the thread but keep the data up to date,
 * they must run the update() function manually from then on.
 */
void OdometryBase::end_async() { this->end_task = true; }

/**
 * Gets the current position and rotation
 */
pose_t OdometryBase::get_position(void) {
  mut.lock();

  // Create a new struct to pass-by-value
  pose_t out = current_pos;

  mut.unlock();

  return out;
}

/**
 * Sets the current position of the robot
 */
void OdometryBase::set_position(const pose_t &newpos) {
  mut.lock();

  current_pos = newpos;

  mut.unlock();
}

AutoCommand *OdometryBase::SetPositionCmd(const pose_t &newpos) {
  return new FunctionCommand([&]() {
    set_position(newpos);
    return true;
  });
}

/**
 * Get the distance between two points
 * @param start_pos distance from this point
 * @param end_pos to this point
 * @return the euclidean distance between start_pos and end_pos
 */
double OdometryBase::pos_diff(pose_t start_pos, pose_t end_pos) {
  // Use the pythagorean theorem
  double retval = sqrt(pow(end_pos.x - start_pos.x, 2) + pow(end_pos.y - start_pos.y, 2));

  return retval;
}

/**
 * Get the change in rotation between two points
 */
double OdometryBase::rot_diff(pose_t pos1, pose_t pos2) { return pos1.rot - pos2.rot; }

/**
 * Get the smallest difference in angle between a start heading and end heading.
 * Returns the difference between -180 degrees and +180 degrees, representing the robot
 * turning left or right, respectively.
 */
double OdometryBase::smallest_angle(double start_deg, double end_deg) {
  double retval;
  // get the difference between 0 and 360
  retval = fmod(end_deg - start_deg, 360.0);
  if (retval < 0) {
    retval += 360.0;
  }

  // Get the closest angle, now between -180 (turn left) and +180 (turn right)
  if (retval > 180) {
    retval -= 360;
  }

  return retval;
}

double OdometryBase::get_speed() {
  mut.lock();
  double retval = speed;
  mut.unlock();

  return retval;
}

double OdometryBase::get_accel() {
  mut.lock();
  double retval = accel;
  mut.unlock();

  return retval;
}

double OdometryBase::get_angular_speed_deg() {
  mut.lock();
  double retval = ang_speed_deg;
  mut.unlock();

  return retval;
}

double OdometryBase::get_angular_accel_deg() {
  mut.lock();
  double retval = ang_accel_deg;
  mut.unlock();

  return retval;
}


pose_t OdometryBase::pose_exponential(const Eigen::Vector3d old_pose, const Eigen::Vector3d twist) {
  double dtheta = twist(2);

  double sinTheta = sin(dtheta);
  double cosTheta = cos(dtheta);

  double sinOldTheta = sin(old_pose(2));
  double cosOldTheta = cos(old_pose(2));

  Eigen::Matrix3d rotation{{cosOldTheta, -sinOldTheta, 0}, {sinOldTheta, cosOldTheta, 0}, {0, 0, 1}};
  Eigen::Matrix3d integrated_rotation;

  // when the angle change is very small, we use a taylor series to approximate
  if (std::abs(dtheta) < 1e-9) {
    integrated_rotation.row(0) << 1.0 - ((dtheta * dtheta) / 6.0), -(dtheta / 2.0), 0;
    integrated_rotation.row(1) << (dtheta / 2.0), 1.0 - ((dtheta * dtheta) / 6.0), 0;
    integrated_rotation.row(2) << 0, 0, 1;
  } else {
    integrated_rotation.row(0) << sinTheta / dtheta, (cosTheta - 1.0) / dtheta, 0;
    integrated_rotation.row(1) << (1 - cosTheta) / dtheta, sinTheta / dtheta, 0;
    integrated_rotation.row(2) << 0, 0, 1;
  }

  Eigen::Vector3d global_pose_delta = rotation * integrated_rotation * twist;

  pose_t newPose;

  newPose.x = old_pose(0) + global_pose_delta(0);
  newPose.y = old_pose(1) + global_pose_delta(1);
  newPose.rot = old_pose(2) + global_pose_delta(2); //wrap_angle_rad(old_pose(2) + global_pose_delta(2));

  return newPose;
}
