#include "dobot_perception/camera_node.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dobot_perception_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  dobot_perception::CameraNode node(nh, private_nh);

  ros::spin();

  return 0;
}
