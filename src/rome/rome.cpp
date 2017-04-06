#include "rome.hpp"

namespace RoME {

  void RoME::AddPose(const Eigen::Vector3d &position,
                     const Eigen::Quaterniond &orientation, const int pose_id) const {
    lcm::LCM lcm_node();
    rome::pose_node_t pose_message;

  }

  void RoME::AddPartialXYH(const Eigen::Vector3d &rel_pose,
                           const Eigen::Matrix3d &Sigma, const int origin_id,
                           const int dest_id, const double confidence ) const {}; 

  void RoME::AddPriorXYH(const Eigen::Vector3d &pose, const Eigen::Matrix3d &Sigma,
                         const int pose_id) const {};

  // this method alone makes this a Caesar client instead of just RoME.
  // void RoME::AddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  //                     const int pose_id) const {};

}



