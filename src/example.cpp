#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Geometry>

struct Observation
{
    ros::Time stamp; // just for debug
    sensor_msgs::LaserScan scan;
    std::map<std::string, Eigen::Affine3d> transformations; // only part that is published to the CEKF filter
};

std::map<std::string, Observation> robots;


void callback(const sensor_msgs::LaserScan::ConstPtr& msg, std::string base_frame, std::string sensor_frame)
{
  ROS_INFO("Recieved scan on callback");
  ROS_INFO_STREAM("Base frame is: " << base_frame.c_str());
  ROS_INFO_STREAM("Sensor frame is: " <<sensor_frame.c_str());
  ROS_ASSERT("sensor_frame not equivalent to frame_id from laser_scan" && sensor_frame != msg->header.frame_id);

  //std::map<std::string, Eigen::Affine3d> other_robot_observations;
  std::vector<Eigen::Affine3d> other_robot_observations;

  std::map<std::string, Eigen::Affine3d> relative_observations;

  for(auto it = robots.begin(); it != robots.end(); ++it) {
    if(it->first == sensor_frame)
      continue; // skip own frame

    ROS_INFO_STREAM("Map is formated as: \n" << it->first);
    for ( const auto &myPair : it->second.transformations) {
        ROS_INFO_STREAM("transformations: " << myPair.first << "\n" << myPair.second.matrix()); // transformations are not used by the callbacks
    }
    // scan accessed like this ...
    sensor_msgs::LaserScan robots_scan = it->second.scan;
    // calculate transform to existing robots ...
    relative_observations[it->first] = Eigen::Affine3d::Identity(); // should be ndt returning this
  }
  robots[sensor_frame] = {ros::Time::now(), *msg, relative_observations};
}

int main( int argc, char** argv ){
 ros::init(argc, argv, "list_parameter_example_node");
 ros::NodeHandle nh;

 XmlRpc::XmlRpcValue robots;
 nh.getParam("robots", robots);
 ROS_ASSERT("failed to load parameters!" && robots.getType() == XmlRpc::XmlRpcValue::TypeStruct);
 std::vector<ros::Subscriber> sub_vector;
 for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = robots.begin(); it != robots.end(); ++it) {
   ROS_INFO_STREAM("Found robots: " << (std::string)(it->first) << " ==> " << robots[it->first]);
   ROS_INFO_STREAM("scan_topic_name: " << robots[it->first]["scan_topic_name"]);
   ROS_INFO_STREAM("Base_frame: " << robots[it->first]["base_frame"]);
   ROS_INFO_STREAM("sensor_frame: " << robots[it->first]["sensor_frame"]);

   sub_vector.push_back(nh.subscribe<sensor_msgs::LaserScan>(robots[it->first]["scan_topic_name"],1,
       boost::bind(&callback, _1, robots[it->first]["base_frame"],robots[it->first]["sensor_frame"])));

 }

 ros::Rate loop_rate(5); //5Hz

 while(ros::ok())
 {
   ros::spinOnce();
   loop_rate.sleep();
 }
}
