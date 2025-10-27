#include "dobot_perception/camera_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

namespace dobot_perception
{
namespace
{
constexpr char kDefaultImageTopic[] = "/camera/color/image_raw";
}

CameraNode::CameraNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
: image_transport_(nh)
{
  private_nh.param<std::string>("image_topic", image_topic_, kDefaultImageTopic);

  image_sub_ = image_transport_.subscribe(
    image_topic_,
    1,
    &CameraNode::imageCallback,
    this);

  ROS_INFO_STREAM("dobot_perception listening for images on topic: " << image_topic_);
}

void CameraNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    const auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    const auto& image = cv_ptr->image;

    const auto stamp = msg->header.stamp;
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Received frame " << image.cols << "x" << image.rows
      << " at " << stamp.sec << "." << stamp.nsec);
  }
  catch (const cv_bridge::Exception& ex)
  {
    ROS_ERROR_STREAM_THROTTLE(2.0, "cv_bridge exception: " << ex.what());
  }
}

}  // namespace dobot_perception
