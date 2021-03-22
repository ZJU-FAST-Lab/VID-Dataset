#pragma once

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace quadrotor_common
{

struct QuadStateEstimate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadStateEstimate();
  QuadStateEstimate(const nav_msgs::Odometry& state_estimate_msg);
  virtual ~QuadStateEstimate();

  nav_msgs::Odometry toRosMessage() const;
  void transformVelocityToWorldFrame();
  bool isValid() const;

  ros::Time timestamp;
  enum class CoordinateFrame
  {
    INVALID, WORLD, OPTITRACK, VISION, LOCAL
  } coordinate_frame;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d bodyrates; // Body rates are represented in body coordinates
};

} // namespace quadrotor_common
