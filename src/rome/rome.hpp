#pragma once

#include <Eigen/Dense>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/rome.hpp>

// #futureinclude neo4j

// #include <pcl/common/common_headers.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>

// let's try and keep this stateless, yes?
// although it would be nice to have a way to get pose updates.
// look at caesar/examples/database/python/neo4j_interact.py for API ideas

namespace RoME {

class Pose {};

class Noise {};

class RoME {

public:
  RoME();
  ~RoME();

  void AddPose(const Eigen::Vector3d &position,
               const Eigen::Quaterniond &orientation, const int pose_id) const;

  void AddPartialXYH(const Eigen::Vector3d &rel_pose,
                     const Eigen::Matrix3d &Sigma, const int origin_id,
                     const int dest_id, const double confidence) const;

  void AddPriorXYH(const Eigen::Vector3d &pose, const Eigen::Matrix3d &Sigma,
                   const int pose_id) const;

  // this method alone makes this a Caesar client instead of just RoME.
  // void AddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  //               const int pose_id) const;

private:
};
};
