#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ekf/ekf.h>

void deserializeVelocities(const char* filename, float& linear, float& angular);
void deserializeObservations(const char* filename, ObservationVector& observations);
void drawRobotTrajectory(const Vector2fVector& positions);
void drawScene(const ObservationVector& observations, const Eigen::Vector3f& mu, const Eigen::Matrix3f& sigma);

//occupancy grid (for visualization)
float resolution = 0.01;
float inv_resolution = 1.f/resolution;
Eigen::Vector2f origin(-17.57, -15.01);
cv::Mat image;

int main(int argc, char** argv){

  //input data
  float linear,angular;
  ObservationVector observations;

  //EKF
  EKF ekf;

  //read data
  int seq=-1;
  std::string line;
  std::ifstream data(argv[1]);
  if(data.is_open()){
    while(std::getline(data,line)){

      //parse line
      seq++;
      std::cerr << "Seq: " << seq << std::endl;
      std::istringstream iss(line);
      double timestamp;
      std::string velocities_filename,observations_filename;
      iss>>timestamp>>velocities_filename>>observations_filename;

      //get velocities
      deserializeVelocities(velocities_filename.c_str(),linear,angular);
      std::cerr << "Read velocities from: " << velocities_filename << std::endl;
      std::cerr << "ux: " << linear << " - utheta: " << angular << std::endl;

      //get observations
      deserializeObservations(observations_filename.c_str(),observations);
      std::cerr << "Read observations from: " << observations_filename << std::endl;
      for(int i=0; i<observations.size(); ++i)
        std::cerr << observations[i]._position.transpose() << std::endl;

      //prediction
      ekf.prediction(linear,angular);

      //correction
      ekf.correction(observations);

      //draw
      drawScene(observations,ekf.mu(),ekf.sigma());
      cv::namedWindow("output",cv::WINDOW_NORMAL);
      cv::imshow("output",image);
      cv::waitKey(0);
    }
  }

  return 0;
}

void deserializeVelocities(const char* filename, float& linear, float& angular){
  std::ifstream fin(filename);
  std::string line;

  if(fin.is_open()){
    if(std::getline(fin,line)){
      std::istringstream iss(line);
      double lx,az;
      iss >>lx>>az;
      linear=lx;
      angular=az;
    }
  }

  fin.close();
}

void deserializeObservations(const char* filename, ObservationVector &observations){
  std::ifstream fin(filename);
  std::string line;

  observations.clear();
  Observation o;
  if(fin.is_open()){
    while(std::getline(fin,line)){
      std::istringstream iss(line);
      std::string id;
      iss>>id;
      double px,py;
      iss>>px>>py;
      o._id = id;
      o._position = Eigen::Vector2f(px,py);
      observations.push_back(o);
    }
  }
  fin.close();
}

void drawScene(const ObservationVector &observations, const Eigen::Vector3f& mu, const Eigen::Matrix3f& sigma){

  //world map (for visualization)
  image = cv::imread("output.png", CV_LOAD_IMAGE_COLOR);

  //observations
  for(int i=0; i<observations.size(); ++i){
    cv::Point obs((observations[i]._position.x()-origin.x())*inv_resolution,
                       image.rows-(observations[i]._position.y()-origin.y())*inv_resolution);
    cv::circle(image,obs,20,cv::Scalar(0,0,255),10);
  }

  //estimated pose
  std::cerr << "Robot pose: " << mu.transpose() << std::endl;
  cv::Point robot((mu.x()-origin.x())*inv_resolution,
                  image.rows-(mu.y()-origin.y())*inv_resolution);
  cv::Point robot_front((cos(mu.z())*0.5+mu.x()-origin.x())*inv_resolution,
                        image.rows-(sin(mu.z())*0.5+mu.y()-origin.y())*inv_resolution);
  cv::circle(image,robot,30,cv::Scalar(255,0,0),-1);
  cv::line(image,robot,robot_front,cv::Scalar(0,255,0),5);

  //estimated covariance
//  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(sigma.block<2,2>(0,0));
//  if (eigensolver.info() != Eigen::Success){
//    std::cerr << "covariance eigenvalues decomposition failed!" << std::endl;
//    abort();
//  }
//  Eigen::Vector2f eigenvalues = eigensolver.eigenvalues();
//  Eigen::Matrix2f eigenvectors = eigensolver.eigenvectors();
////  std::cerr << "Eigen values: " << eigenvalues.transpose() << std::endl;
////  std::cerr << "Eigen vectors: " << std::endl << eigenvectors << std::endl;
//  int i_max = (eigenvalues(0) > eigenvalues(1) ? 0 : 1);
//  int i_min = (eigenvalues(0) > eigenvalues(1) ? 1 : 0);
//  Eigen::Vector2f axis = mu.head(2)+eigenvectors.col(i_max)*eigenvalues(i_max);
//  double rotation = atan2(axis.y()-mu.y(),axis.x()-mu.x());
//  cv::Size axes(sqrt(eigenvalues(i_max))/2.0f,sqrt(eigenvalues(i_min))/2.0f);
//  cv::ellipse(image,robot,axes,rotation,0.0,360.0,cv::Scalar(0,0,255));
}
