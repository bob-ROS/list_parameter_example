#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/bind.hpp>

void callback(const sensor_msgs::LaserScan::ConstPtr& msg, std::string base_frame, std::string sensor_frame)
{
  ROS_INFO("Recieved scan on callback");
  ROS_INFO_STREAM("Base frame is: " << base_frame.c_str());
  ROS_INFO_STREAM("Sensor frame is: " <<sensor_frame.c_str());
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
