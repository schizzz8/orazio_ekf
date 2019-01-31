#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

#include <ekf/ekf.h>


class MessageDumperNode{

    typedef std::map<std::string,Eigen::Vector3f, std::less<std::string>,
    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3f> > > StringVector3fMap;

  public:
    MessageDumperNode(ros::NodeHandle nh_,const std::string& filename):
      _nh(nh_){

      _twist_sub = _nh.subscribe("/lucrezio/cmd_vel",1000,&MessageDumperNode::filterCallback,this);

      _out.open(filename);
      _seq=-1;

      ROS_INFO("Starting data dumper node...");
      readLandmarks();
    }

    void filterCallback(const geometry_msgs::Twist::ConstPtr& twist_msg){
      _seq++;
      _last_timestamp = ros::Time::now();

      //serialize velocities
      char velocities_filename[80];
      sprintf(velocities_filename,"twist_%lu.txt",_seq);
      serializeVelocities(velocities_filename,*twist_msg);

      //listen to robot pose
      tf::StampedTransform robot_tf;
      try{
        _listener.waitForTransform("/map",
                                   "/base_footprint",
                                   _last_timestamp,
                                   ros::Duration(0.5));
        _listener.lookupTransform("/map",
                                  "/base_footprint",
                                  _last_timestamp,
                                  robot_tf);
      } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      Eigen::Isometry3f robot_transform = tfTransform2eigen(robot_tf);

      //serialize ground_truth
      char ground_truth_filename[80];
      sprintf(ground_truth_filename,"ground_truth_%lu.txt",_seq);
      serializeTransform(ground_truth_filename,robot_transform);


      //serialize observations
      char observations_filename[80];
      sprintf(observations_filename,"observations_%lu.txt",_seq);
      generateObservations(observations_filename,robot_transform);

      //write to output file
      _out << _last_timestamp << " ";
      _out << ground_truth_filename << " ";
      _out << velocities_filename << " ";
      _out << observations_filename << std::endl;

      //heart beat
      std::cerr << ".";
    }

  protected:
    ros::NodeHandle _nh;
    ros::Time _last_timestamp;

    ros::Subscriber _twist_sub;
    tf::TransformListener _listener;

    std::ofstream _out;
    size_t _seq;

    StringVector3fMap _landmarks;

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

    Eigen::Isometry3f poseMsg2eigen(const geometry_msgs::Pose &p){
      Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
      iso.translation().x()=p.position.x;
      iso.translation().y()=p.position.y;
      iso.translation().z()=p.position.z;
      Eigen::Quaternionf q;
      q.x()=p.orientation.x;
      q.y()=p.orientation.y;
      q.z()=p.orientation.z;
      q.w()=p.orientation.w;
      iso.linear()=q.toRotationMatrix();
      return iso;
    }

    void serializeTransform(const char* filename, const Eigen::Isometry3f &transform){
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

    void serializeVelocities(const char* filename, const geometry_msgs::Twist& twist){
      std::ofstream data;
      data.open(filename);

      data << twist.linear.x << " "
           << twist.angular.z << std::endl;

      data.close();
    }

    void readLandmarks(){
      std::string filename = "/home/dede/source/lucrezio/lucrezio_simulation_environments/config/envs/orazio_world/object_locations.yaml";

      YAML::Node map = YAML::LoadFile(filename);
      for(YAML::const_iterator it=map.begin(); it!=map.end(); ++it){
        const std::string &key=it->first.as<std::string>();

        Eigen::Vector3f pos;
        YAML::Node attributes = it->second;
        YAML::Node position = attributes["position"];
        for(int i=0; i<position.size(); ++i){
          pos(i) = position[i].as<float>();
        }

        _landmarks.insert(std::make_pair(key,pos));
      }
    }

    void generateObservations(const char* filename, const Eigen::Isometry3f& T){
      std::ofstream data;
      data.open(filename);

      Eigen::Isometry3f inv_T = T.inverse();
      for(StringVector3fMap::iterator it=_landmarks.begin(); it!=_landmarks.end(); ++it){
        const std::string landmark_id = it->first;
        data << landmark_id << " ";
        Eigen::Vector3f landmark_position = it->second;
        landmark_position = inv_T*landmark_position;
        data << landmark_position.x() << " " << landmark_position.y() << std::endl;
      }
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
