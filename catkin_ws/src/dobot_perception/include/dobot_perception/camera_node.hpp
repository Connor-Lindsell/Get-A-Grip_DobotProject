#ifndef DOBOT_PERCEPTION_CAMERA_NODE_HPP
#define DOBOT_PERCEPTION_CAMERA_NODE_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

namespace dobot_perception
{

class CameraNode
{
public:
  CameraNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  std::string image_topic_;
};

}  // namespace dobot_perception

#endif  // DOBOT_PERCEPTION_CAMERA_NODE_HPP
