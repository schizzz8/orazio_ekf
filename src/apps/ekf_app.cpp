#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <object_detector/model.h>

typedef std::vector<Model> ModelVector;
typedef std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

void deserializeTransform(const char * filename, Eigen::Isometry3f &transform);
void deserializeModels(const char * filename, ModelVector & models);
void drawRobotTrajectory(const Vector2fVector& positions);

int main(int argc, char** argv){

  Eigen::Isometry3f odom_transform = Eigen::Isometry3f::Identity();
  ModelVector landmarks;

  //read data
  bool first=true;
  int seq=-1;
  std::string line;
  std::ifstream data(argv[1]);
  if(data.is_open()){
    if(std::getline(data,line)){

    }
  }

  return 0;
}

void deserializeTransform(const char * filename, Eigen::Isometry3f &transform){
  std::ifstream fin(filename);
  std::string line;

  transform.setIdentity();
  if(fin.is_open()){
    if(std::getline(fin,line)){
      std::istringstream iss(line);
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      transform.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      transform.linear().matrix() = R;
    }
  }

  fin.close();
}

void deserializeModels(const char * filename, ModelVector & models){
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

void drawRobotTrajectory(const Vector2fVector& positions){
 float resolution = 0.01;
 Eigen::Vector2f origin(-17.57, -15.01);
 cv::Mat image;
 image = cv::imread("output.png", CV_LOAD_IMAGE_COLOR);
 cv::Point last_cell((0-origin.x())/resolution,image.rows-(0-origin.y())/resolution);

 for(const Eigen::Vector2f& pos : positions){
   cv::Point cell((pos.x()-origin.x())/resolution,image.rows-(pos.y()-origin.y())/resolution);
   cv::circle(image, cell, 1, cv::Scalar(255,0,0));
   cv::line(image, last_cell, cell, cv::Scalar(0,0,255));
   last_cell = cell;
 }
 cv::imshow("output",image);
 cv::waitKey();

 cv::imwrite("trajectory.png",image);
}
