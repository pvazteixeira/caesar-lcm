#include "caesar.hpp"

namespace Caesar {

void Caesar::AddPose(const Pose3d &pose, const int pose_id) const {
  caesar::pose_node_t pose_message;

  pose_message.utime = 0;
  pose_message.id = pose_id;
  pose_message.mean_dim = 7;
  std::vector<double> pose_mean = {pose.x(),  pose.y(),  pose.z(), pose.qw(),
                                   pose.qx(), pose.qy(), pose.qz()};
  pose_message.mean = pose_mean;
  pose_message.covar_dim = 6;
  std::vector<double> covar_diag = {1.0e-2, 1.0e-2, 1.0e-2,
                                    1.0e-2, 1.0e-2, 1.0e-2};
  pose_message.covar = covar_diag;

  lcm::LCM lcm_node;
  lcm_node.publish("CAESAR_POSES", &pose_message);
}

void Caesar::AddOdometry(const Pose3d &delta_pose, const int origin_id,
                         const int destination_id) const {

  Eigen::VectorXd covar;
  // default odometry: std dev of 0.1 m in XYZ and 5.7 degrees in YPR
  covar << 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2;
  AddOdometry(delta_pose, covar, origin_id, destination_id);
}

void Caesar::AddOdometry(const Pose3d &delta_pose,
                         const Eigen::VectorXd &covars, const int origin_id,
                         const int destination_id) const {
  caesar::pose_pose_nh_t message;
  message.utime = 0;

  message.node_1_utime = 0;
  message.node_1_id = origin_id;

  message.node_2_utime = 0;
  message.node_2_id = destination_id;

  message.mean_dim = 7;
  std::vector<double> odometry = delta_pose.Vector();
  message.mean = odometry;

  message.covar_dim = 6;

  std::vector<double> covar_diag = {covars(0), covars(1), covars(2),
                                    covars(3), covars(4), covars(5)};
  message.covar = covar_diag;

  message.confidence = 1.0;

  lcm::LCM lcm_node;
  lcm_node.publish("CAESAR_FACTORS", &message);
};

void Caesar::AddPartialXYH(const Eigen::Vector3d &rel_pose, const int origin_id,
                           const int dest_id) const {
  Eigen::Vector3d covars;
  covars << 1.0e-2, 1.0e-2, 1.0e-2;
  AddPartialXYH(rel_pose, covars, origin_id, dest_id);
}

void Caesar::AddPartialXYH(const Eigen::Vector3d &rel_pose,
                           const Eigen::Vector3d &covars, const int origin_id,
                           const int dest_id) const {
  // rename AddPartial2D(...)?
  caesar::pose_pose_xyh_t message;

  message.utime = 0;
  message.node_1_utime = message.node_2_utime = 0;
  message.node_1_id = origin_id;
  message.node_2_id = dest_id;
  message.delta_x = rel_pose.x();
  message.delta_y = rel_pose.y();
  message.delta_yaw = rel_pose.z();

  message.var_x = covars(0);
  message.var_y = covars(1);
  message.var_yaw = covars(2);

  lcm::LCM lcm_node;
  lcm_node.publish("CAESAR_PARTIAL_XYH", &message);
};

void Caesar::AddPartialXYHNH(const Eigen::Vector3d &rel_pose,
                             const int origin_id, const int dest_id,
                             const double confidence) const {
  Eigen::Vector3d covars;
  covars << 1.0e-2, 1.0e-2, 1.0e-2;
  AddPartialXYHNH(rel_pose, covars, origin_id, dest_id);
}

void Caesar::AddPartialXYHNH(const Eigen::Vector3d &rel_pose,
                             const Eigen::Vector3d &covars, const int origin_id,
                             const int dest_id, const double confidence) const {
  // rename AddPartial2D(...)?
  caesar::pose_pose_xyh_nh_t message;

  message.utime = 0;
  message.node_1_utime = message.node_2_utime = 0;
  message.node_1_id = origin_id;
  message.node_2_id = dest_id;
  message.delta_x = rel_pose.x();
  message.delta_y = rel_pose.y();
  message.delta_yaw = rel_pose.z();

  message.var_x = covars(0);
  message.var_y = covars(1);
  message.var_yaw = covars(2);

  message.confidence = confidence;

  lcm::LCM lcm_node;
  lcm_node.publish("CAESAR_PARTIAL_XYH_NH", &message);
};

void Caesar::AddPriorZPR(const Eigen::Vector3d &v, const int pose_id) const {
  Eigen::Vector3d covars;
  // std dev of .01 m in Z, ~.57 degrees in roll, pitch
  covars << 1.0e-4, 1.0e-4, 1.0e-4;
  AddPriorZPR(v,covars,pose_id);
}

void Caesar::AddPriorZPR(const Eigen::Vector3d &v, const Eigen::Vector3d &vars,
                         const int pose_id) const {
  // adds a prior in Z, pitch, and roll
  caesar::prior_zpr_t message;

  message.utime = 0;
  message.id = pose_id;

  message.z = v.x();
  message.pitch = v.y();
  message.roll = v.z();

  message.var_z = vars(0);
  message.var_pitch = vars(1);
  message.var_roll = vars(2);

  lcm::LCM lcm_node;
  lcm_node.publish("CAESAR_PARTIAL_ZPR", &message);
};

void Caesar::AddPose3Pose3NH(const Eigen::Vector3d &rel_position,
                             const Eigen::Quaterniond &rel_orientation,
                             const int origin_id, const int dest_id,
                             const double confidence) const {
  caesar::pose_pose_nh_t message;

  message.utime = 0;
  message.node_1_utime = 0;
  message.node_1_id = origin_id;

  message.node_2_utime = 0;
  message.node_2_id = dest_id;

  message.mean_dim = 7;
  std::vector<double> mean = {rel_position.x(),    rel_position.y(),
                              rel_position.z(),    rel_orientation.w(),
                              rel_orientation.x(), rel_orientation.y(),
                              rel_orientation.z()};
  message.mean = mean;

  message.covar_dim = 6;
  std::vector<double> covar_diag = {1.0e-2, 1.0e-2, 1.0e-2,
                                    1.0e-2, 1.0e-2, 1.0e-2};
  message.covar = covar_diag;

  message.confidence = confidence;

  lcm::LCM lcm_node;
  lcm_node.publish("CAESAR_P3P3NH", &message);
};

void Caesar::AddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                      const int pose_id) const {
  caesar::point_cloud_t point_cloud_message;

  point_cloud_message.utime = 0;
  point_cloud_message.id = pose_id;

  point_cloud_message.n = cloud->points.size();
  for (int i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZRGB pt = cloud->points[i];
    std::vector<double> ptv = {pt.x, pt.y, pt.z};
    std::vector<uint8_t> ptc = {pt.r, pt.g, pt.b};
    point_cloud_message.points.push_back(ptv);
    point_cloud_message.colors.push_back(ptc);
  }

  lcm::LCM lcm_node;
  lcm_node.publish("CAESAR_POINT_CLOUDS", &point_cloud_message);
};
}
