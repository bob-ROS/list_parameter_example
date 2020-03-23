#include <ros/ros.h>
#include <XmlRpcValue.h>

int main( int argc, char** argv ){
 ros::init(argc, argv, "list_parameter_example_node");
 ros::NodeHandle nh;

 XmlRpc::XmlRpcValue robots;
 nh.getParam("robots", robots);
 ROS_ASSERT("failed to load parameters!" && robots.getType() == XmlRpc::XmlRpcValue::TypeStruct);
 for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = robots.begin(); it != robots.end(); ++it) {
   ROS_INFO_STREAM("Found robots: " << (std::string)(it->first) << " ==> " << robots[it->first]);
   ROS_INFO_STREAM("Base_link: " << robots[it->first]["base_frame"]);
   ROS_INFO_STREAM("sensor_frame: " << robots[it->first]["sensor_frame"]);
 }
}
