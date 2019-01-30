#include "ekf.h"

EKF::EKF():
  _mu(Eigen::Vector3f::Zero()),
  _sigma(Eigen::Matrix3f::Identity()*0.001){
  _landmarks.clear();
  EKF::readLandmarks();
}

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

void EKF::correction(const ObservationVector& observations){
  if(observations.empty())
    return;

  const int n_obs = observations.size();

  //current estimate of the state
  Eigen::Isometry2f T = EKF::v2t(_mu);
  Eigen::Isometry2f inv_T = T.inverse();

  //useful variables
  Eigen::Vector2f t = T.translation();
  Eigen::Matrix2f Rt = inv_T.linear();
  Eigen::Matrix2f dRt = EKF::dRt(_mu.z());

  //process observations
  Eigen::Vector2f lt;
  Eigen::MatrixXf zt = Eigen::MatrixXf::Zero(2*n_obs,1);
  Eigen::MatrixXf ht = Eigen::MatrixXf::Zero(2*n_obs,1);
  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(2*n_obs,3);
  for(int i=0; i<n_obs; ++i){

    //actual measurement zt
    zt.block<2,1>(i*2,0) = observations[i]._position;

    //predicted measurement ht
    lt = _landmarks[observations[i]._id];
    ht.block<2,1>(i*2,0) = inv_T*lt;

    //build Jacobian
    C.block<2,2>(i*2,0) = -Rt;
    C.block<2,1>(i*2,2) = dRt*(lt-t);
  }

  //observation noise
  float noise = 0.01;
  Eigen::MatrixXf sigma_z = Eigen::MatrixXf::Identity(2*n_obs,2*n_obs)*noise;

  //Kalman gain
  Eigen::MatrixXf K = Eigen::MatrixXf::Zero(3,2*n_obs);
  K = _sigma*C.transpose()*(C*_sigma*C.transpose()+sigma_z).inverse();

  //update mu
  _mu += K*(zt-ht);

  //update sigma
  _sigma = (Eigen::Matrix3f::Identity() - K*C)*_sigma;
}

Eigen::Vector3f EKF::transitionModel(Eigen::Vector3f mu, const float& ux, const float& utheta){
  Eigen::Vector3f mu_prime = Eigen::Vector3f::Zero();

  mu_prime.x() = mu.x() + ux*cos(mu.z());
  mu_prime.y() = mu.y() + ux*sin(mu.z());
  mu_prime.z() = mu.z() + utheta;

  return mu_prime;
}

Eigen::Isometry2f EKF::v2t(const Eigen::Vector3f &v){
  Eigen::Isometry2f T = Eigen::Isometry2f::Identity();
  T.translation() = Eigen::Vector2f(v.x(),v.y());
  T.linear() = Eigen::Rotation2Df(v.z()).matrix();
  return T;
}

Eigen::Vector3f EKF::t2v(const Eigen::Isometry2f &T){
  Eigen::Vector3f v = Eigen::Vector3f::Zero();
  v.head(2) = T.translation();
  Eigen::Matrix2f R = T.linear();
  v(2) = atan2(R(1,0),R(0,0));
}

Eigen::Matrix2f EKF::dRt(const float &a){
  Eigen::Matrix2f dRt = Eigen::Matrix2f::Zero();
  Eigen::Matrix2f dR;
  dR << -sin(a),-cos(a),cos(a),-sin(a);
  dRt = dR.transpose();
  return dRt;
}

void EKF::readLandmarks(){
  std::string filename = "/home/dede/source/lucrezio/lucrezio_simulation_environments/config/envs/orazio_world/object_locations.yaml";

  YAML::Node map = YAML::LoadFile(filename);
  for(YAML::const_iterator it=map.begin(); it!=map.end(); ++it){
    const std::string &key=it->first.as<std::string>();

    Eigen::Vector2f pos;
    YAML::Node attributes = it->second;
    YAML::Node position = attributes["position"];
    for(int i=0; i<2; ++i){
      pos(i) = position[i].as<float>();
    }

    _landmarks.insert(std::make_pair(key,pos));
  }
}
