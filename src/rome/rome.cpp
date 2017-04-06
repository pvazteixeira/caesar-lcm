#include "rome.hpp"

namespace RoME {

void RoME::AddPose(const Pose3d &pose, const int pose_id) const {
  rome::pose_node_t pose_message;

  pose_message.utime = 0;
  pose_message.id = pose_id;
  pose_message.mean_dim = 7;
  std::vector<double> pose_mean = {pose.x(),  pose.y(),  pose.z(), pose.qw(),
                                   pose.qx(), pose.qy(), pose.qz()};
  pose_message.mean = pose_mean;
  pose_message.covar_dim = 6;
  std::vector<double> covar_diag = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  pose_message.covar = covar_diag;

  lcm::LCM lcm_node;
  lcm_node.publish("ROME_POSES", &pose_message);
}

void RoME::AddPartialXYH(const Pose2d &rel_pose, const int origin_id,
                         const int dest_id) const {
    // rename AddPartial2D(...)?
};

void RoME::AddPriorZPR(double z, double pitch, double roll,
                       const Eigen::Matrix3d &Sigma, const int pose_id) const {
    // adds a prior in Z, pitch, and roll
};

void RoME::AddPose3Pose3NH(const Eigen::Vector3d &rel_position,
                           const Eigen::Quaterniond &rel_orientation,
                           const Eigen::MatrixXd &Sigma, const int origin_id,
                           const int dest_id) const {
    // will be implemented
};

void RoME::AddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    const int pose_id) const {
  rome::point_cloud_t point_cloud_message;

  point_cloud_message.utime = 0;
  point_cloud_message.id = pose_id;

  point_cloud_message.n = cloud->points.size();
  for (int i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZRGB pt = cloud->points[i];
  std:
    std::vector<double> ptv = {pt.x, pt.y, pt.z};
    std::vector<uint8_t> ptc = {pt.r, pt.g, pt.b};
    point_cloud_message.points.push_back(ptv);
    point_cloud_message.colors.push_back(ptc);
  }

  lcm::LCM lcm_node;
  lcm_node.publish("ROME_POINT_CLOUDS", &point_cloud_message);
};
}
