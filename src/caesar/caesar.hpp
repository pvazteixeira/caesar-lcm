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

  /*!
   * \brief Add a pose node.
   *
   * \param [in] pose The initial pose estimate.
   * \param [in] pose_id Pose node ID.
   */
  void AddPose(const Pose3d &pose, const int pose_id) const;

  /*!
   * \brief Add an odometry factor.
   *
   * \param [in] delta_pose Odometry vector.
   * \param [in] origin_id Origin node ID.
   * \param [in] destination_id Destination node ID.
   */
  void AddOdometry(const Pose3d &delta_pose, const int origin_id,
                   const int destination_id) const;

  /*!
   * \brief Add an odometry factor
   *
   * \param [in] delta_pose Odometry vector.
   * \param [in] covars Covariance matrix diagonal.
   * \param [in] origin_id Origin node ID.
   * \param [in] destination_id Destination node ID.
   */
  void AddOdometry(const Pose3d &delta_pose, const Eigen::VectorXd &covars,
                   const int origin_id, const int destination_id) const;

  /*!
   * \brief Add a partial constraint in X, Y, and heading.
   *
   * \param [in] rel_pose Measurement vector.
   * \param [in] origin_id Origin node ID.
   * \param [in] dest_id Destination node ID.
   */
  void AddPartialXYH(const Eigen::Vector3d &rel_pose, const int origin_id,
                     const int dest_id) const;

  /*!
   * \brief Add a partial constraint in X, Y, and heading.
   *
   * \param [in] rel_pose Measurement vector.
   * \param [in] covars Covariance matrix diagonal.
   * \param [in] origin_id Origin node ID.
   * \param [in] dest_id Destination node ID.
   */
  void AddPartialXYH(const Eigen::Vector3d &rel_pose,
                     const Eigen::Vector3d &covars, const int origin_id,
                     const int dest_id) const;

  /*!
   * \brief Add a partial constraint in X, Y, and heading, with null hypothesis support.
   *
   * \param [in] rel_pose Measurement vector.
   * \param [in] origin_id Origin node ID.
   * \param [in] dest_id Destination node ID.
   * \param [in] confidence Probability of a valid constraint.
   */
  void AddPartialXYHNH(const Eigen::Vector3d &rel_pose, const int origin_id,
                       const int dest_id, const double confidence = 0.5) const;

  /*!
   * \brief Add a partial constraint in X, Y, and heading, with null hypothesis support.
   *
   * \param [in] rel_pose Measurement vector.
   * \param [in] covars Covariance matrix diagonal.
   * \param [in] origin_id Origin node ID.
   * \param [in] dest_id Destination node ID.
   * \param [in] confidence Probability of a valid constraint.
   */
  void AddPartialXYHNH(const Eigen::Vector3d &rel_pose,
                       const Eigen::Vector3d &covars, const int origin_id,
                       const int dest_id, const double confidence = 0.5) const;

  /*!
   * \brief Add a partial constraint in Z, pitch, and roll.
   *
   * \param [in] v Measurement vector.
   * \param [in] pose_id Node id.
   */
  void AddPriorZPR(const Eigen::Vector3d &v, const int pose_id) const;

  /*!
   * \brief Add a partial constraint in Z, pitch, and roll.
   *
   * \param [in] v Measurement vector.
   * \param [in] covar Measurement covariance matrix diagonal.
   * \param [in] pose_id Node id.
   */
  void AddPriorZPR(const Eigen::Vector3d &v, const Eigen::Vector3d &covar,
                   const int pose_id) const;

  /*!
   * \brief
   *
   * \param [in] rel_position
   * \param [in] rel_orientation
   * \param [in] origin_id
   * \param [in] dest_id
   * \param [in] confidence
   */
  void AddPose3Pose3NH(const Eigen::Vector3d &rel_position,
                       const Eigen::Quaterniond &rel_orientation,
                       const int origin_id, const int dest_id,
                       const double confidence = 0.5) const;

  /*!
   * \brief Add a point cloud to a pose node.
   *
   * \param [in] cloud Point cloud.
   * \param [in] pose_id Node id.
   */
  void AddCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                const int pose_id) const;

private:
};
};
