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
[debug] Adding submap to map...i=0:
        Eigen::Vector3d pos1(8.7481, 7.51, 4.5662);
        Eigen::Quaterniond q1(0.9939, -0.0280019, -0.00824962, -0.106355);

        Eigen::Vector3d pos2(9.43, 11.0879, 4.9);
        Eigen::Quaterniond q2(0.992387, 0.0575732, -0.00698004, -0.108645);
  Eigen::Vector3 xyh2(-0.0884074, 3.64126, 0; -0.00556528, 0, 0);
        Eigen::Vector3 zpr2(      4.9,-0.00134375,   0.116047);

        Eigen::Vector3d pos3(9.94, 15.1484, 4.8792)
        Eigen::Quaterniond q3(0.997597, 0.0563353, -0.0115003, -0.0386678
  Eigen::Vector3 xyh3(-0.380935, 4.0746, 0; 0.139626, 0, 0);
        Eigen::Vector3 zpr3(  4.8792,-0.0185897,  0.113553);

        Eigen::Vector3d pos4(9.93104, 14.5831, 5.8)
        Eigen::Quaterniond q4(0.997626, 0.0336346, -0.011705, -0.0589371
  Eigen::Vector3 xyh4(0.0354154, -0.564254, 0; -0.0401426, 0, 0)
        Eigen::Vector3 zpr4(    5.8,-0.019391,0.0685557);

        Eigen::Vector3d pos5(9.12, 10.1615, 5.84927)
        Eigen::Quaterniond q5(0.998351, 0.0362801, -0.00973871, -0.0434122
  Eigen::Vector3 xyh5(-0.281789, -4.48658, 0; 0.0311713, 0, 0)
        Eigen::Vector3 zpr5(5.84927,-0.016296,0.0733617);
*/

int main(int argCount, char **argValues) {

  // OK
  Eigen::Vector3d pos1(8.7481, 7.51, 4.5662);
  Eigen::Quaterniond q1(0.9939, -0.0280019, -0.00824962, -0.106355);

  // OK
  Eigen::Vector3d pos2(9.43, 11.0879, 4.9);
  Eigen::Quaterniond q2(0.992387, 0.0575732, -0.00698004, -0.108645);
  // 
  Eigen::Vector3d xyh2(-0.0884074, 3.64126, -0.00556528);
  Eigen::Vector3d zpr2(4.9, -0.00134375, 0.116047);

  // OK
  Eigen::Vector3d pos3(9.94, 15.1484, 4.8792);
  Eigen::Quaterniond q3(0.997597, 0.0563353, -0.0115003, -0.0386678);
  // 
  Eigen::Vector3d xyh3(-0.380935, 4.0746, 0.139626);
  Eigen::Vector3d zpr3(4.8792, -0.0185897, 0.113553);

  // OK
  Eigen::Vector3d pos4(9.93104, 14.5831, 5.8);
  Eigen::Quaterniond q4(0.997626, 0.0336346, -0.011705, -0.0589371);
  Eigen::Vector3d xyh4(0.0354154, -0.564254, -0.0401426);
  Eigen::Vector3d zpr4(5.8, -0.019391, 0.0685557);

  // OK
  Eigen::Vector3d pos5(9.12, 10.1615, 5.84927);
  Eigen::Quaterniond q5(0.998351, 0.0362801, -0.00973871, -0.0434122);
  //
  Eigen::Vector3d xyh5(-0.281789, -4.48658, 0.0311713);
  Eigen::Vector3d zpr5(5.84927, -0.016296, 0.0733617);

  std::vector<Eigen::Vector3d> position = {pos1, pos2, pos3, pos4, pos5};
  std::vector<Eigen::Quaterniond> orientation = {q1, q2, q3, q4, q5};
  std::vector<Eigen::Vector3d> prior = {zpr2, zpr3, zpr4, zpr5};
  std::vector<Eigen::Vector3d> odometry = {xyh2, xyh3, xyh4, xyh5};

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  
  RoME::RoME rome;

  for (int i = 0; i < 5; ++i) {

    int id = i + 1;
    std::clog << "Adding pose " << i << "...";
    RoME::Pose3d pose(position[i], orientation[i]);
    rome.AddPose(pose, id);
    std::clog << "done.\n";

    if (i > 0) {
      std::clog << "Adding ZPR prior...";
      rome.AddPriorZPR(prior[i - 1], id);
      std::clog << "done.\n";

      std::clog << "Adding odometry...";
      rome.AddPartialXYH(odometry[i - 1], id - 1, id);
      std::clog << "done.\n";
    }

    std::clog << "Adding cloud " << i << "...";
    std::string filename = "../../data/" + std::to_string(i) + ".pcd";
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud);
    rome.AddCloud(cloud, i + 1);
    std::clog << "done.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // for (int i = 0; i < 5; ++i) {
  //   std::clog << "Adding cloud " << i << "...";
  //   std::string filename = "../../data/" + std::to_string(i) + ".pcd";
  //   pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud);
  //   rome.AddCloud(cloud, i + 1);
  //   std::clog << "done.\n";
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // }

  // we're done here
  std::clog << "Exiting.\n\n";
}
