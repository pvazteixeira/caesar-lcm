#include <chrono>
#include <iostream>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/rome.hpp>

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <rome/rome.hpp>

/*
0:
Pose3d: (18.6099, 2.22993, 6.87; 2.08567, -0.0300105, 0.148694)
t = 18.6099, 2.22993, 6.87
q = 0.503288, 0.0244895, -0.0716953, 0.860791

1:
Pose3d: (13.9014, 0.0904617, 6.85; 2.11352, -0.00637565, 0.167041)
t = 13.9014, 0.0904617, 6.85
q = 0.490211, 0.0382526, -0.074204, 0.867596
XYH:   0.45566   5.08974      0.0240488
ZPR:   6.85      -0.00637565  0.167041

2:
Pose3d: (12.1801, -0.43, 7.79993; 2.10485, -0.0118666, 0.163867)
t = 12.1801, -0.43, 7.79993
q = 0.494217, 0.0354126, -0.0740187, 0.865457
XYH:    0.449375     1.87598 -0.00763192
ZPR:    7.79993 -0.0118666   0.163867

3:
Pose3d: (15.7593, 1.16965, 7.82; 2.04727, -0.0165574, 0.0860389)
t = 15.7593, 1.16965, 7.82
q = 0.52007, 0.0153104, -0.0410293, 0.853
XYH:  -0.444757   -3.83878 -0.0560354
ZPR:       7.82 -0.0165574  0.0860389

4:
Pose3d: (19.8287, 3.11935, 7.83065; 2.10138, -0.00818067, 0.104991)
t = 19.8287, 3.11935, 7.83065
q = 0.49647, 0.0225321, -0.0475625, 0.866457
XYH: -0.133697  -4.49257 0.0531888
ZPR:     7.83065 -0.00818067    0.104991
 */

int main(int argCount, char **argValues) {

  RoME::RoME rome;

  std::clog << "Adding pose 1...";
  RoME::Pose3d pose_1(16.3, 1.15, 5.78, 0.478507, 0.0306824, -0.0719457,
                      0.874593);
  rome.AddOdometry(pose_1, 1, 1);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Adding pose 2...";
  RoME::Pose3d pose_2(-0.341546, -2.64716, 0.412176, 0.996592, 0.0414649,
                      0.01563, 0.069579);
  rome.AddOdometry(pose_2, 1, 2);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Adding pose 3...";
  RoME::Pose3d pose_3(-0.687755, -2.45876, 0.570817, 0.998815, -0.0171313,
                      0.0152455, 0.0429151);
  rome.AddOdometry(pose_3, 2, 3);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Adding pose 4...";
  RoME::Pose3d pose_4(-1.14993, -2.85719, 0.579899, 0.997835, -0.0112134,
                      -0.0134271, -0.0633934);
  rome.AddOdometry(pose_4, 3, 4);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Adding cloud 1...";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../data/0.pcd", *cloud1);
  rome.AddCloud(cloud1, 1);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Adding cloud 2...";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../data/1.pcd", *cloud2);
  rome.AddCloud(cloud2, 2);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Adding cloud 3...";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(
                                                new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../data/2.pcd", *cloud3);
  rome.AddCloud(cloud3, 3);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Adding cloud 4...";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4(
                                                new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../data/3.pcd", *cloud4);
  rome.AddCloud(cloud4, 4);
  std::clog << "done.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::clog << "Exiting.\n\n";
}
