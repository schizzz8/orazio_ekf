#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ekf/ekf.h>

void deserializeTwist(const char* filename, Eigen::Vector3f& linear, Eigen::Vector3f& angular);
void deserializeLandmarks(const char * filename, Vector2fVector& landmarks);
void deserializeObservations(const char* filename, Vector2fVector& observations);
void drawRobotTrajectory(const Vector2fVector& positions);
void drawScene(const Vector2fVector& landmarks, const Eigen::Vector3f& mu, const Eigen::Matrix3f& sigma);

//occupancy grid (for visualization)
float resolution = 0.01;
float inv_resolution = 1.f/resolution;
Eigen::Vector2f origin(-17.57, -15.01);
cv::Mat image;

int main(int argc, char** argv){

  //input data
  Eigen::Vector3f linear = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular = Eigen::Vector3f::Zero();
  Vector2fVector landmarks,observations;

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
      std::string twist_filename,landmarks_filename,observations_filename;
      iss>>timestamp>>twist_filename>>landmarks_filename>>observations_filename;

      //get odometry
      deserializeTwist(twist_filename.c_str(),linear,angular);
      std::cerr << "Read odometry from: " << twist_filename << std::endl;
      std::cerr << "ux: " << linear.x() << " - utheta: " << angular.z() << std::endl;

      //get landmarks
      deserializeLandmarks(landmarks_filename.c_str(),landmarks);
      std::cerr << "Read landmarks from: " << landmarks_filename << std::endl;
      for(int i=0; i<landmarks.size(); ++i)
        std::cerr << landmarks[i].transpose() << std::endl;

      //get observations
      deserializeObservations(observations_filename.c_str(),observations);
      std::cerr << "Read observations from: " << observations_filename << std::endl;
      for(int i=0; i<observations.size(); ++i)
        std::cerr << observations[i].transpose() << std::endl;

      //prediction
      ekf.prediction(linear.x(),angular.z());

      //correction
      ekf.correction(landmarks,observations);

      //draw
      drawScene(landmarks,ekf.mu(),ekf.sigma());
      cv::namedWindow("output",cv::WINDOW_NORMAL);
      cv::imshow("output",image);
      cv::waitKey(0);
    }
  }

  return 0;
}

void deserializeTwist(const char* filename, Eigen::Vector3f& linear, Eigen::Vector3f& angular){
  std::ifstream fin(filename);
  std::string line;

  if(fin.is_open()){
    if(std::getline(fin,line)){
      std::istringstream iss(line);
      double lx,ly,lz,ax,ay,az;
      iss >>lx>>ly>>lz>>ax>>ay>>az;
      linear=Eigen::Vector3f(lx,ly,lz);
      angular=Eigen::Vector3f(ax,ay,az);
    }
  }

  fin.close();
}

void deserializeLandmarks(const char * filename, Vector2fVector &landmarks){
  std::ifstream fin(filename);
  std::string line;

  landmarks.clear();
  if(fin.is_open()){
    while(std::getline(fin,line)){
      std::istringstream iss(line);
      double px,py;
      iss>>px>>py;
      landmarks.push_back(Eigen::Vector2f(px,py));
    }
  }

  fin.close();
}

void deserializeObservations(const char* filename, Vector2fVector &observations){
  std::ifstream fin(filename);
  std::string line;

  observations.clear();
  if(fin.is_open()){
    while(std::getline(fin,line)){
      std::istringstream iss(line);
      double px,py;
      iss>>px>>py;
      observations.push_back(Eigen::Vector2f(px,py));
    }
  }
  fin.close();
}

void drawScene(const Vector2fVector &landmarks, const Eigen::Vector3f& mu, const Eigen::Matrix3f& sigma){

  //world map (for visualization)
  image = cv::imread("output.png", CV_LOAD_IMAGE_COLOR);

  //landmarks
  std::cerr << "Landmarks:" << std::endl;
  for(int i=0; i<landmarks.size(); ++i){
    std::cerr << i << ": " << landmarks[i].transpose() << std::endl;
    cv::Point landmark((landmarks[i].x()-origin.x())*inv_resolution,
                       image.rows-(landmarks[i].y()-origin.y())*inv_resolution);
    cv::circle(image,landmark,20,cv::Scalar(0,0,255),10);
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
