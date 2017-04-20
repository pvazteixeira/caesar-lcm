#pragma once

#include <Eigen/Dense>

namespace Caesar {

class Pose2d {
public:
  Pose2d() {
    position_ << 0.0, 0.0;
    yaw_ = 0.0;
  };

  Pose2d(double x, double y, double yaw) {
    position_ << x, y;
    yaw_ = yaw;
  };

  ~Pose2d(){};

  double x() const {return position_.x();}; 

  double y() const {return position_.y();}; 
  
  double yaw() const { return yaw_;};

private:
  Eigen::Vector2d position_;
  double yaw_;
};
};
