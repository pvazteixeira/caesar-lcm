#pragma once

#include <Eigen/Dense>

#include <lcm/lcm-cpp.hpp>

#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include "Pose3d.hpp"
#include <lcmtypes/rome.hpp>
// let's try and keep this stateless, yes?
// although it would be nice to have a way to get pose updates.
// look at caesar/examples/database/python/neo4j_interact.py for API ideas
// [[file+emacs:~/.julia/v0.5/Caesar/examples/database/python/neo4j_interact.py][neo4j_interact.py]]

namespace RoME {

class Pose2d {
public:
  Pose2d() {
    position_ << 0.0, 0.0;
    yaw_ = 0.0;
  };
  Pose2d(double x, double y, double yaw) {
    position_ << x, y;
    yaw_ = yaw;
  };

private:
  Eigen::Vector2d position_;
  double yaw_;
};

class Noise {};

class RoME {

public:
  RoME(){};
  ~RoME(){};

  void AddPose(const Pose3d &pose, const int pose_id) const;

  void AddPartialXYH(const Pose2d &rel_pose, const int origin_id,
                     const int dest_id) const;

  void AddPriorZPR(double z, double pitch, double roll,
                   const Eigen::Matrix3d &Sigma, const int pose_id) const;

  void AddPose3Pose3NH(const Eigen::Vector3d &rel_position,
                       const Eigen::Quaterniond &rel_orientation,
                       const Eigen::MatrixXd &Sigma, const int origin_id,
                       const int dest_id) const;

  // this method alone makes this a Caesar client instead of just RoME.
  void AddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                const int pose_id) const;

private:
};
};
