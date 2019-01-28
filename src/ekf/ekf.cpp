#include "ekf.h"

EKF::EKF():
  _mu(Eigen::Vector3f::Zero()),
  _sigma(Eigen::Matrix3f::Identity()*0.001){}

void EKF::prediction(const float &ux, const float &utheta){

  //transition model
  Eigen::Vector3f mu_prime = transitionModel(_mu,ux,utheta);

  //Jacobian A
  Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
  A(0,2) = -ux*sin(_mu.z());
  A(1,2) = ux*cos(_mu.z());

  //Jacobian A
  Eigen::MatrixXf B = Eigen::MatrixXf::Zero(3,2);
  B(0,0) = cos(_mu.z());
  B(1,0) = sin(_mu.z());
  B(2,1) = 1;

  //motion noise
  Eigen::Matrix2f sigma_u = Eigen::Matrix2f::Zero();
  float noise = 0.1; //constant part
  float v_noise = pow(ux,2); //lin vel dependent part
  float w_noise = pow(utheta,2); //ang vel dependent part
  sigma_u(0,0) = pow(noise,2) + v_noise;
  sigma_u(1,1) = pow(noise,2) + w_noise;

  //predict mu
  _mu = mu_prime;

  //predict sigma
  _sigma = A*_sigma*A.transpose() + B*sigma_u*B.transpose();
}

void EKF::correction(const ModelVector &landmarks, const Vector2fVector &observations){
  if(observations.empty())
    return;


}

Eigen::Vector3f EKF::transitionModel(Eigen::Vector3f mu, const float& ux, const float& utheta){
  Eigen::Vector3f mu_prime = Eigen::Vector3f::Zero();

  mu_prime.x() = mu.x() + ux*cos(mu.z());
  mu_prime.y() = mu.y() + ux*sin(mu.z());
  mu_prime.z() = mu.z() + utheta;

  return mu_prime;
}
