#pragma once
#include <iostream>
#include <map>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

typedef std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
typedef std::map<std::string,Eigen::Vector2f, std::less<std::string>,
Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector2f> > > StringVector2fMap;

struct Observation{
    std::string _id;
    Eigen::Vector2f _position;
};
typedef std::vector<Observation> ObservationVector;

class EKF{
  public:
    EKF();

    const Eigen::Vector3f& mu() const {return _mu;}
    const Eigen::Matrix3f& sigma() const {return _sigma;}

    void prediction(const float& ux, const float& utheta);

    void correction(const ObservationVector& observations);


  protected:
    Eigen::Vector3f _mu;
    Eigen::Matrix3f _sigma;

    StringVector2fMap _landmarks;

  private:
    void readLandmarks();

    Eigen::Vector3f transitionModel(Eigen::Vector3f mu, const float& ux, const float& utheta);

    Eigen::Isometry2f v2t(const Eigen::Vector3f& v);
    Eigen::Vector3f t2v(const Eigen::Isometry2f& T);
    Eigen::Matrix2f dRt(const float& a);
};
