#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <lucrezio_simulation_environments/LogicalImage.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

typedef std::vector<lucrezio_simulation_environments::Model> Models;

class MessageDumperNode{
  public:
    MessageDumperNode(ros::NodeHandle nh_,const std::string& filename):
      _nh(nh_){
      _logical_sub = _nh.subscribe("/gazebo/logical_camera_image",10,&MessageDumperNode::logicalCallback,this);

      _out.open(filename);
      _seq=-1;

      ROS_INFO("Starting data dumper node...");
    }

    void logicalCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr& logical_image_msg){

      //check that the there's at list one landmark in the robot field-of-view
      if(logical_image_msg->models.empty()){
        ROS_ERROR("No landmarks observed!!!");
        return;
      }

      //store last timestamp
      _seq++;
      _last_timestamp = logical_image_msg->header.stamp;

      //save odometry
      std::string error;
      if (! _listener.waitForTransform ("/odom",
                                        "/base_footprint",
                                        _last_timestamp,
                                        ros::Duration(0.5),
                                        ros::Duration(0.5),
                                        &error)) {
        std::cerr << "MessageDumper: transform error from /odom to /base_footprint: " << error << std::endl;
        return;
      }
      tf::StampedTransform odom_pose_t;
      _listener.lookupTransform("/odom",
                                "/base_footprint",
                                _last_timestamp,
                                odom_pose_t);
      const Eigen::Isometry3f odom_transform=tfTransform2eigen(odom_pose_t);
      char transform_filename[80];
      sprintf(transform_filename,"odom_transform_%lu.txt",_seq);
      serializeTransform(transform_filename,odom_transform);


      //save landmarks
      const Models &models=logical_image_msg->models;
      char models_filename[80];
      sprintf(models_filename,"models_%lu.txt",_seq);
      serializeModels(models_filename,models);

      //write to output file
      _out << _last_timestamp << " ";
      _out << transform_filename << " ";
      _out << models_filename << std::endl;

      std::cerr << ".";
    }

  protected:
    ros::NodeHandle _nh;
    ros::Time _last_timestamp;
    ros::Subscriber _logical_sub;
    tf::TransformListener _listener;

    std::ofstream _out;
    size_t _seq;

  private:

    Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p){
      Eigen::Isometry3f iso;
      iso.translation().x()=p.getOrigin().x();
      iso.translation().y()=p.getOrigin().y();
      iso.translation().z()=p.getOrigin().z();
      Eigen::Quaternionf q;
      tf::Quaternion tq = p.getRotation();
      q.x()= tq.x();
      q.y()= tq.y();
      q.z()= tq.z();
      q.w()= tq.w();
      iso.linear()=q.toRotationMatrix();
      return iso;
    }

    void serializeModels(char* filename, const Models &models){
      std::ofstream data;
      data.open(filename);

      int num_models=models.size();
      data << num_models << std::endl;

      for(int i=0; i<num_models; ++i){
        const lucrezio_simulation_environments::Model &model = models[i];
        data << model.type << " ";
        tf::StampedTransform model_pose;
        tf::poseMsgToTF(model.pose,model_pose);
        const Eigen::Isometry3f model_transform=tfTransform2eigen(model_pose);
        data << model_transform.translation().x() << " "
             << model_transform.translation().y() << " "
             << model_transform.translation().z() << " ";

        const Eigen::Matrix3f model_rotation = model_transform.linear().matrix();
        data << model_rotation(0,0) << " "
             << model_rotation(0,1) << " "
             << model_rotation(0,2) << " "
             << model_rotation(1,0) << " "
             << model_rotation(1,1) << " "
             << model_rotation(1,2) << " "
             << model_rotation(2,0) << " "
             << model_rotation(2,1) << " "
             << model_rotation(2,2) << " ";

        data << model.min.x << " "
             << model.min.y << " "
             << model.min.z << " "
             << model.max.x << " "
             << model.max.y << " "
             << model.max.z << std::endl;

      }
      data.close();
    }

    void serializeTransform(char* filename, const Eigen::Isometry3f &transform){
      std::ofstream data;
      data.open(filename);

      data << transform.translation().x() << " "
           << transform.translation().y() << " "
           << transform.translation().z() << " ";

      const Eigen::Matrix3f rotation = transform.linear().matrix();
      data << rotation(0,0) << " "
           << rotation(0,1) << " "
           << rotation(0,2) << " "
           << rotation(1,0) << " "
           << rotation(1,1) << " "
           << rotation(1,2) << " "
           << rotation(2,0) << " "
           << rotation(2,1) << " "
           << rotation(2,2) << std::endl;

      data.close();

    }
};

int main(int argc, char** argv){

  ros::init(argc, argv, "message_dumper_node");
  ros::NodeHandle nh;

  MessageDumperNode dumper(nh,argv[1]);

  ros::spin();

  std::cerr << std::endl;


  return 0;
}
