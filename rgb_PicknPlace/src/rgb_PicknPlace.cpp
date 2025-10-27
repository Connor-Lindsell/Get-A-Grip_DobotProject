#include "rgb_PicknPlace/rgb_PicknPlace.h"

#include <algorithm>
#include <boost/bind.hpp>
#include <cctype>
#include <cmath>
#include <limits>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rgb_picknplace
{
namespace
{
    bool isDepthEncodingSupported(const std::string& encoding)
    {
        return encoding == sensor_msgs::image_encodings::TYPE_16UC1 || encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    }
}  // namespace

RgbPicknPlaceNode::RgbPicknPlaceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , busy_(false)
{
  loadParameters();

  ee_pub_ = nh_.advertise<geometry_msgs::Pose>("/dobot_magician/target_end_effector_pose", 1, true);
  tool_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/dobot_magician/target_tool_state", 1, true);

  const std::string color_topic = pnh_.param<std::string>("color_topic", "/camera/color/image_raw");
  const std::string depth_topic = pnh_.param<std::string>("depth_topic", "/camera/depth/image_rect_raw");
  const int sync_queue = pnh_.param("sync_queue", 5);

  if (!isDepthEncodingSupported(sensor_msgs::image_encodings::TYPE_16UC1))
  {
    ROS_DEBUG("Depth encoding support table initialised.");
  }

  color_sub_.subscribe(nh_, color_topic, 1);
  depth_sub_.subscribe(nh_, depth_topic, 1);
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(sync_queue), color_sub_, depth_sub_));
  sync_->registerCallback(boost::bind(&RgbPicknPlaceNode::cameraCallback, this, _1, _2));

  start_srv_ = pnh_.advertiseService("start", &RgbPicknPlaceNode::startService, this);
  stop_srv_ = pnh_.advertiseService("stop", &RgbPicknPlaceNode::stopService, this);
  resume_srv_ = pnh_.advertiseService("resume", &RgbPicknPlaceNode::resumeService, this);
  estop_srv_ = pnh_.advertiseService("estop", &RgbPicknPlaceNode::estopService, this);

  enabled_ = pnh_.param("auto_start", false);
  if (enabled_)
  {
    ROS_INFO("Auto-start enabled: processing RealSense frames immediately.");
  }
  else
  {
    ROS_INFO("Node initialised in stopped state. Call the start service to begin classification.");
  }

  ROS_INFO_STREAM("rgb_PicknPlace node ready. Subscribed to colour topic '" << color_topic
                  << "' and depth topic '" << depth_topic << "'.");
}

// Load parameters from the parameter server
void RgbPicknPlaceNode::loadParameters()
{
  home_pose_ = getPoseParam("home_pose", {0.2, 0.0, 0.15, 0.0, 0.0, 0.0});
  pre_pick_pose_ = getPoseParam("pre_pick_pose", {0.25, 0.0, 0.10, 0.0, 0.0, 0.0});
  pick_pose_ = getPoseParam("pick_pose", {0.25, 0.0, 0.025, 0.0, 0.0, 0.0});

  double dwell_seconds = pnh_.param("dwell_time", 0.75);
  dwell_time_ = ros::Duration(dwell_seconds);

  double cooldown_seconds = pnh_.param("detection_cooldown", detection_cooldown_.toSec());
  detection_cooldown_ = ros::Duration(cooldown_seconds);

  pnh_.param("roi_width", roi_width_, roi_width_);
  pnh_.param("roi_height", roi_height_, roi_height_);
  pnh_.param("min_saturation", min_saturation_, min_saturation_);
  pnh_.param("min_value", min_value_, min_value_);
  pnh_.param("depth_scale", depth_scale_, depth_scale_);
  pnh_.param("adjust_pick_height", adjust_pick_height_, adjust_pick_height_);
  pnh_.param("pick_surface_offset", pick_surface_offset_, pick_surface_offset_);
  pnh_.param("min_pick_height", min_pick_height_, min_pick_height_);
  pnh_.param("max_pick_height", max_pick_height_, max_pick_height_);

  drop_sites_["red"] = getPlacementSiteParam("red", {{0.10, 0.18, 0.10, 0.0, 0.0, 0.0}, {0.10, 0.18, 0.03, 0.0, 0.0, 0.0}});
  drop_sites_["green"] = getPlacementSiteParam("green", {{0.18, -0.18, 0.10, 0.0, 0.0, 0.0}, {0.18, -0.18, 0.03, 0.0, 0.0, 0.0}});
  drop_sites_["blue"] = getPlacementSiteParam("blue", {{0.28, 0.10, 0.10, 0.0, 0.0, 0.0}, {0.28, 0.10, 0.03, 0.0, 0.0, 0.0}});
}

// Get a PoseRPY parameter from the parameter server
PoseRPY RgbPicknPlaceNode::getPoseParam(const std::string& name, const PoseRPY& fallback)
{
  PoseRPY pose = fallback;
  pnh_.param(name + "/x", pose.x, fallback.x);
  pnh_.param(name + "/y", pose.y, fallback.y);
  pnh_.param(name + "/z", pose.z, fallback.z);
  pnh_.param(name + "/roll", pose.roll, fallback.roll);
  pnh_.param(name + "/pitch", pose.pitch, fallback.pitch);
  pnh_.param(name + "/yaw", pose.yaw, fallback.yaw);
  return pose;
}

// Get placement site parameters for a given colour
PlacementSite RgbPicknPlaceNode::getPlacementSiteParam(const std::string& color, const PlacementSite& fallback)
{
  PlacementSite site = fallback;
  site.approach = getPoseParam(color + "_approach", fallback.approach);
  site.place = getPoseParam(color + "_place", fallback.place);
  return site;
}

// =======================================================================
// Service methods
// =======================================================================

// Check if the node can accept a new pick-and-place cycle
bool RgbPicknPlaceNode::canAcceptWork(std::string* reason) const
{
  if (estop_active_)
  {
    if (reason)
    {
      *reason = "estop engaged";
    }
    return false;
  }

  if (!enabled_)
  {
    if (reason)
    {
      *reason = "node stopped";
    }
    return false;
  }

  if (busy_)
  {
    if (reason)
    {
      *reason = "cycle already in progress";
    }
    return false;
  }

  if (!last_detection_time_.isZero())
  {
    const ros::Duration since_last = ros::Time::now() - last_detection_time_;
    if (since_last < detection_cooldown_)
    {
      if (reason)
      {
        *reason = "cooldown active";
      }
      return false;
    }
  }

  return true;
}

// Enable or disable processing
void RgbPicknPlaceNode::setEnabled(bool enabled)
{
  if (enabled == enabled_)
  {
    return;
  }

  enabled_ = enabled;
  if (enabled_)
  {
    ROS_INFO("rgb_PicknPlace state: STARTED");
  }
  else
  {
    ROS_INFO("rgb_PicknPlace state: STOPPED");
  }
}

// Start processing frames
bool RgbPicknPlaceNode::startService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& resp)
{
  if (estop_active_)
  {
    resp.success = false;
    resp.message = "Cannot start while estop is engaged. Call resume to clear.";
    return true;
  }

  setEnabled(true);
  resp.success = true;
  resp.message = "Started colour classification.";
  return true;
}

// Stop after completing any in-flight cycle
bool RgbPicknPlaceNode::stopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& resp)
{
  setEnabled(false);
  resp.success = true;
  resp.message = "Stopped after completing any in-flight cycle.";
  return true;
}

// Resume from estop or normal stop
bool RgbPicknPlaceNode::resumeService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& resp)
{
  if (estop_active_)
  {
    estop_active_ = false;
    ROS_WARN("E-stop cleared via resume service.");
  }

  setEnabled(true);
  resp.success = true;
  resp.message = "Resumed colour classification.";
  return true;
}

// Emergency stop: immediately disable processing
bool RgbPicknPlaceNode::estopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& resp)
{
  estop_active_ = true;
  setEnabled(false);
  resp.success = true;
  resp.message = "Emergency stop engaged. Call resume to re-enable.";
  return true;
}

// ========================================================================
// Callback and processing methods
// ========================================================================

void RgbPicknPlaceNode::cameraCallback(const sensor_msgs::ImageConstPtr& color_msg,
                                       const sensor_msgs::ImageConstPtr& depth_msg)
{
  std::string reason;
  if (!canAcceptWork(&reason))
  {
    ROS_DEBUG_STREAM_THROTTLE(2.0, "Skipping frame: " << reason);
    return;
  }

  cv_bridge::CvImageConstPtr color_ptr;
  try
  {
    color_ptr = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception& ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "cv_bridge failed to convert colour image: " << ex.what());
    return;
  }

  const cv::Rect roi = computeRoi(color_ptr->image);
  if (roi.width <= 0 || roi.height <= 0)
  {
    ROS_WARN_THROTTLE(2.0, "ROI is empty after bounding by image dimensions.");
    return;
  }

  const std::string colour = classifyColor(color_ptr->image(roi));
  if (colour.empty())
  {
    ROS_DEBUG_THROTTLE(5.0, "No confident colour classification this frame.");
    return;
  }

  auto site_it = drop_sites_.find(colour);
  if (site_it == drop_sites_.end())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Received colour '" << colour << "' without a configured drop site. Ignoring.");
    return;
  }

  double pick_height = pick_pose_.z;
  if (adjust_pick_height_)
  {
    const double depth = extractDepthMeters(depth_msg, roi);
    if (std::isfinite(depth))
    {
      pick_height = std::clamp(depth - pick_surface_offset_, min_pick_height_, max_pick_height_);
    }
  }

  busy_ = true;
  last_detection_time_ = ros::Time::now();

  ROS_INFO_STREAM("Detected colour '" << colour << "'. Starting pick-and-place cycle.");

  executePickAndPlace(site_it->second, pick_height);

  ROS_INFO_STREAM("Finished cycle for colour '" << colour << "'.");
  busy_ = false;
}

std::string RgbPicknPlaceNode::classifyColor(const cv::Mat& roi_bgr) const
{
  if (roi_bgr.empty())
  {
    return {};
  }

  cv::Mat hsv;
  cv::cvtColor(roi_bgr, hsv, cv::COLOR_BGR2HSV);
  const cv::Scalar mean_hsv = cv::mean(hsv);
  const double hue = mean_hsv[0];
  const double sat = mean_hsv[1];
  const double val = mean_hsv[2];

  if (sat < min_saturation_ || val < min_value_)
  {
    return {};
  }

  if (hue < 15.0 || hue >= 165.0)
  {
    return "red";
  }
  if (hue >= 35.0 && hue <= 85.0)
  {
    return "green";
  }
  if (hue >= 90.0 && hue <= 140.0)
  {
    return "blue";
  }

  return {};
}

cv::Rect RgbPicknPlaceNode::computeRoi(const cv::Mat& image) const
{
  if (image.empty())
  {
    return {};
  }

  const int width = std::min(roi_width_, image.cols);
  const int height = std::min(roi_height_, image.rows);
  const int x = std::max(0, (image.cols - width) / 2);
  const int y = std::max(0, (image.rows - height) / 2);

  return cv::Rect(x, y, width, height);
}

double RgbPicknPlaceNode::extractDepthMeters(const sensor_msgs::ImageConstPtr& depth_msg, const cv::Rect& roi) const
{
  if (!depth_msg)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (!isDepthEncodingSupported(depth_msg->encoding))
  {
    ROS_WARN_STREAM_THROTTLE(5.0, "Unsupported depth encoding: " << depth_msg->encoding);
    return std::numeric_limits<double>::quiet_NaN();
  }

  cv_bridge::CvImageConstPtr depth_ptr;
  try
  {
    depth_ptr = cv_bridge::toCvShare(depth_msg);
  }
  catch (const cv_bridge::Exception& ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "cv_bridge failed to convert depth image: " << ex.what());
    return std::numeric_limits<double>::quiet_NaN();
  }

  cv::Rect bounded_roi = roi & cv::Rect(0, 0, depth_ptr->image.cols, depth_ptr->image.rows);
  if (bounded_roi.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const cv::Mat roi_depth = depth_ptr->image(bounded_roi);
  double sum = 0.0;
  int count = 0;

  if (roi_depth.type() == CV_16UC1)
  {
    for (int r = 0; r < roi_depth.rows; ++r)
    {
      const uint16_t* row = roi_depth.ptr<uint16_t>(r);
      for (int c = 0; c < roi_depth.cols; ++c)
      {
        const uint16_t raw = row[c];
        if (raw == 0)
        {
          continue;
        }
        sum += static_cast<double>(raw) * depth_scale_;
        ++count;
      }
    }
  }
  else if (roi_depth.type() == CV_32FC1)
  {
    for (int r = 0; r < roi_depth.rows; ++r)
    {
      const float* row = roi_depth.ptr<float>(r);
      for (int c = 0; c < roi_depth.cols; ++c)
      {
        const float depth = row[c];
        if (!std::isfinite(depth) || depth <= 0.0f)
        {
          continue;
        }
        sum += static_cast<double>(depth);
        ++count;
      }
    }
  }
  else
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (count == 0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return sum / static_cast<double>(count);
}

void RgbPicknPlaceNode::executePickAndPlace(const PlacementSite& site, double pick_height)
{
  openGripper();

  moveToPose(home_pose_);
  moveToPose(pre_pick_pose_);

  PoseRPY pick_pose = pick_pose_;
  pick_pose.z = pick_height;
  moveToPose(pick_pose);

  closeGripper();
  waitForMotion();

  moveToPose(pre_pick_pose_);
  moveToPose(site.approach);
  moveToPose(site.place);

  openGripper();
  waitForMotion();

  moveToPose(site.approach);
  moveToPose(home_pose_);
}

void RgbPicknPlaceNode::moveToPose(const PoseRPY& pose)
{
  geometry_msgs::Pose msg;
  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.position.z = pose.z;

  tf2::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);
  msg.orientation.w = q.getW();
  msg.orientation.x = q.getX();
  msg.orientation.y = q.getY();
  msg.orientation.z = q.getZ();

  ee_pub_.publish(msg);
  waitForMotion();
}

void RgbPicknPlaceNode::openGripper()
{
  std_msgs::Int32MultiArray tool_msg;
  tool_msg.data = {1, 0};
  tool_pub_.publish(tool_msg);
}

void RgbPicknPlaceNode::closeGripper()
{
  std_msgs::Int32MultiArray tool_msg;
  tool_msg.data = {1, 1};
  tool_pub_.publish(tool_msg);
}

void RgbPicknPlaceNode::waitForMotion()
{
  dwell_time_.sleep();
  ros::spinOnce();
}

}  // namespace rgb_picknplace