#pragma once

#include <Eigen/Geometry>
#include <object_detector/model.h>

typedef std::vector<Model> ModelVector;
typedef std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

class EKF{
  public:
    EKF();

    const Eigen::Vector3f& mu() const {return _mu;}
    const Eigen::Matrix3f& sigma() const {return _sigma;}

    void prediction(const float& ux, const float& utheta);

    void correction(const ModelVector& landmarks, const Vector2fVector& observations);

  protected:
    Eigen::Vector3f _mu;
    Eigen::Matrix3f _sigma;

  private:
    Eigen::Vector3f transitionModel(Eigen::Vector3f mu, const float& ux, const float& utheta);
};
