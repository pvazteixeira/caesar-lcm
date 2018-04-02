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

namespace Caesar {

class Caesar {

public:
  Caesar(){};
  ~Caesar(){};

  void AddPose(const Pose3d &pose, const int pose_id) const;

  void AddOdometry(const Pose3d &delta_pose, const int origin_id,
                   const int destination_id) const;

  void AddOdometry(const Pose3d &delta_pose, const Eigen::VectorXd &covars,
                   const int origin_id, const int destination_id) const;

  void AddPartialXYH(const Eigen::Vector3d &rel_pose, const int origin_id,
                     const int dest_id) const;

  void AddPartialXYH(const Eigen::Vector3d &rel_pose,
                     const Eigen::Vector3d &covars, const int origin_id,
                     const int dest_id) const;

  void AddPartialXYHNH(const Eigen::Vector3d &rel_pose, const int origin_id,
                       const int dest_id, const double confidence = 0.5) const;

  void AddPartialXYHNH(const Eigen::Vector3d &rel_pose,
                       const Eigen::Vector3d &covars, const int origin_id,
                       const int dest_id, const double confidence = 0.5) const;

  void AddPriorZPR(const Eigen::Vector3d &v, const int pose_id) const;

  void AddPriorZPR(const Eigen::Vector3d &v, const Eigen::Vector3d &covar,
                   const int pose_id) const;

  void AddPose3Pose3NH(const Eigen::Vector3d &rel_position,
                       const Eigen::Quaterniond &rel_orientation,
                       const int origin_id, const int dest_id,
                       const double confidence = 0.5) const;

  void AddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                const int pose_id) const;

private:
};
};
