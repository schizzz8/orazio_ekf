#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ekf/ekf.h>

void deserializeTwist(const char* filename, Eigen::Vector3f& linear, Eigen::Vector3f& angular);
void deserializeLandmarks(const char * filename, ModelVector & models);
void deserializeObservations(const char* filename, Vector2fVector& observations);
void drawRobotTrajectory(const Vector2fVector& positions);
void drawScene(const Eigen::Isometry3f& odom_transform, const ModelVector& landmarks);

//occupancy grid (for visualization)
float resolution = 0.01;
float inv_resolution = 1.f/resolution;
Eigen::Vector2f origin(-17.57, -15.01);
cv::Mat image;

int main(int argc, char** argv){

  //world map (for visualization)
  image = cv::imread("output.png", CV_LOAD_IMAGE_COLOR);

  //input data
  Eigen::Vector3f linear = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular = Eigen::Vector3f::Zero();
  ModelVector landmarks;
  Vector2fVector observations;

  //EKF
  EKF ekf;

  //read data
  bool first=true;
  int seq=-1;
  std::string line;
  std::ifstream data(argv[1]);
  if(data.is_open()){
    if(std::getline(data,line)){

      //parse line
      seq++;
      std::cerr << "Seq: " << seq << std::endl;
      std::istringstream iss(line);
      double timestamp;
      std::string twist_filename,landmarks_filename,observations_filename;
      iss>>timestamp>>twist_filename>>landmarks_filename>>observations_filename;

      //get odometry
      deserializeTwist(twist_filename.c_str(),linear,angular);

      //get landmarks
      deserializeLandmarks(landmarks_filename.c_str(),landmarks);

      //get observations
      deserializeObservations(observations_filename.c_str(),observations);

      //draw
//      drawScene(odom_transform,landmarks);
//      cv::namedWindow("output",cv::WINDOW_NORMAL);
//      cv::imshow("output",image);
//      cv::waitKey();
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

void deserializeLandmarks(const char * filename, ModelVector & models){
  std::ifstream fin(filename);
  std::string line;

  models.clear();
  if(fin.is_open()){
    while(std::getline(fin,line)){
      std::istringstream iss(line);
      std::string type;
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      double minx,miny,minz,maxx,maxy,maxz;
      iss >> type;

      if(type.length() < 3)
        continue;

      Eigen::Isometry3f model_pose=Eigen::Isometry3f::Identity();
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      model_pose.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      model_pose.linear().matrix() = R;
      iss >> minx>>miny>>minz>>maxx>>maxy>>maxz;
      Eigen::Vector3f min(minx,miny,minz);
      Eigen::Vector3f max(maxx,maxy,maxz);

      models.push_back(Model(type,model_pose,min,max));
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

void drawScene(const Eigen::Isometry3f& odom_transform, const ModelVector& landmarks){

  //robot
  cv::Point robot((odom_transform.translation().x()-origin.x())*inv_resolution,
      image.rows-(odom_transform.translation().y()-origin.y())*inv_resolution);
  Eigen::Vector3f euler = odom_transform.linear().eulerAngles(0,1,2);
  cv::Point robot_front((cos(euler.z())*0.5+odom_transform.translation().x()-origin.x())*inv_resolution,
                        image.rows-(sin(euler.z())*0.5+odom_transform.translation().y()-origin.y())*inv_resolution);
  cv::circle(image,robot,30,cv::Scalar(255,0,0),-1);
  cv::line(image,robot,robot_front,cv::Scalar(0,255,0),5);

  //landmarks
  for(int i=0; i<landmarks.size(); ++i){
    cv::Point landmark((landmarks[i].position().x()-origin.x())*inv_resolution,
                       image.rows-(landmarks[i].position().y()-origin.y())*inv_resolution);
    cv::circle(image,landmark,20,cv::Scalar(0,0,255),10);
  }
}
