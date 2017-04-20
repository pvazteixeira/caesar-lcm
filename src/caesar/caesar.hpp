#pragma once

#include <Eigen/Dense>

#include <lcm/lcm-cpp.hpp>

#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include "Pose2d.hpp"
#include "Pose3d.hpp"
#include <lcmtypes/caesar.hpp>
// let's try and keep this stateless, yes?
// although it would be nice to have a way to get pose updates.
// look at caesar/examples/database/python/neo4j_interact.py for API ideas
// [[file+emacs:~/.julia/v0.5/Caesar/examples/database/python/neo4j_interact.py][neo4j_interact.py]]

namespace Caesar {

class Caesar {

public:
  Caesar(){};
  ~Caesar(){};

  void AddPose(const Pose3d &pose, const int pose_id) const;

  void AddOdometry(const Pose3d &delta_pose, const int origin_id,
                   const int destination_id) const;

  void AddPartialXYH(const Eigen::Vector3d &rel_pose, const int origin_id,
                           const int dest_id) const ;

  void AddPriorZPR(const Eigen::Vector3d &v, const int pose_id) const ;

  void AddPose3Pose3NH(const Eigen::Vector3d &rel_position,
                       const Eigen::Quaterniond &rel_orientation,
                       const Eigen::MatrixXd &Sigma, const int origin_id,
                       const int dest_id) const;

  void AddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                const int pose_id) const;

private:
};
};
