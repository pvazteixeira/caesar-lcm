namespace RoME{

  class Pose3d {
  public:
    Pose3d() {
      position_ << 0.0, 0.0, 0.0;
      orientation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    };

    Pose3d(double x, double y, double z, double qw, double qx, double qy,
           double qz) {
      position_ << x, y, z;
      orientation_ = Eigen::Quaterniond(qw, qx, qy, qz);
    };

    Pose3d(const Eigen::Vector3d &position,
           const Eigen::Quaterniond &orientation) {
      position_ = position;
      orientation_ = orientation;
    };

    ~Pose3d(){};

    Eigen::Vector3d position() { return position_;};
    double x() const {return position_.x();};
    double y() const {return position_.y();};
    double z() const {return position_.z();};

    double qw() const {return orientation_.w();};
    double qx() const {return orientation_.x();};
    double qy() const {return orientation_.y();};
    double qz() const {return orientation_.z();};

    Eigen::Quaterniond q() const { return orientation_;};

  private:
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
  };
}
