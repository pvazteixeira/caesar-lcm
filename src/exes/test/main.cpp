#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/rome.hpp>

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <rome/rome.hpp>

int main(int argCount, char **argValues) {

  RoME::RoME rome;

  std::clog << "Adding pose 1...";
  RoME::Pose3d pose_1(19.4299, 2.29964, 3.8303, 0.38004, -0.299951, -0.561802,
                      0.670804);
  rome.AddPose(pose_1, 1);
  std::clog << "done.\n";

  // std::clog << "Adding ZPR prior to pose 1...";
  // std::clog << "done.\n";
  
  std::clog << "Adding pose 2...";
  RoME::Pose3d pose_2(21.806, 3.39461, 3.80669, 0.390637, -0.345967, -0.582213,
                      0.623488);
  rome.AddPose(pose_2, 2);
  std::clog << "done.\n";

  // std::clog << "Adding ZPR prior to pose 2...";
  // std::clog << "done.\n";

  // std::clog << "Adding XYH odometry betwen poses 1 and 2...";
  // std::clog << "done.\n";

  std::clog << "Adding cloud 1...";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../data/cloud1.pcd", *cloud1);
  rome.AddCloud(cloud1, 1);
  std::clog << "done.\n";

  std::clog << "Adding cloud 2...";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../data/cloud2.pcd", *cloud2);
  rome.AddCloud(cloud2, 2);
  std::clog << "done.\n";

  std::clog << "Exiting.\n\n";
}
