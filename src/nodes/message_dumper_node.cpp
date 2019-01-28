#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <lucrezio_simulation_environments/LogicalImage.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

typedef std::vector<lucrezio_simulation_environments::Model> Landmarks;

class MessageDumperNode{
  public:
    MessageDumperNode(ros::NodeHandle nh_,const std::string& filename):
      _nh(nh_),
      _logical_image_sub(_nh,"/gazebo/logical_camera_image",30),
      _twist_sub(_nh,"/lucrezio/odom",30),
      _synchronizer(FilterSyncPolicy(1000),_logical_image_sub,_twist_sub){

      _synchronizer.registerCallback(boost::bind(&MessageDumperNode::filterCallback, this, _1, _2));

      _out.open(filename);
      _seq=-1;

      ROS_INFO("Starting data dumper node...");
    }

    void filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr& logical_image_msg,
                        const nav_msgs::Odometry::ConstPtr& nav_msg){

      //check that the there's at list one landmark in the robot field-of-view
      if(logical_image_msg->models.empty()){
        ROS_ERROR("No landmarks observed!!!");
        return;
      }

      //store last timestamp
      _seq++;
      _last_timestamp = logical_image_msg->header.stamp;

      //serialize twist
      char twist_filename[80];
      sprintf(twist_filename,"twist_%lu.txt",_seq);
      serializeTwist(twist_filename,nav_msg->twist.twist);

      //serialize landmarks
      const Landmarks &landmarks=logical_image_msg->models;
      char landmarks_filename[80];
      sprintf(landmarks_filename,"landmarks_%lu.txt",_seq);
      serializeLandmarks(landmarks_filename,landmarks);

      //write to output file
      _out << _last_timestamp << " ";
      _out << twist_filename << " ";
      _out << landmarks_filename << std::endl;


      //heart beat
      std::cerr << ".";
    }

  protected:
    ros::NodeHandle _nh;
    ros::Time _last_timestamp;

    //synchronized subscriber to logical_image and twist
    message_filters::Subscriber<lucrezio_simulation_environments::LogicalImage> _logical_image_sub;
    message_filters::Subscriber<nav_msgs::Odometry> _twist_sub;
    typedef message_filters::sync_policies::ApproximateTime<lucrezio_simulation_environments::LogicalImage,
    nav_msgs::Odometry> FilterSyncPolicy;
    message_filters::Synchronizer<FilterSyncPolicy> _synchronizer;

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

    void serializeLandmarks(char* filename, const Landmarks &models){
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

    void serializeTwist(char* filename, const geometry_msgs::Twist& twist){
      std::ofstream data;
      data.open(filename);

      data << twist.linear.x << " "
           << twist.linear.y << " "
           << twist.linear.z << " "
           << twist.angular.x << " "
           << twist.angular.y << " "
           << twist.angular.z << std::endl;

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
