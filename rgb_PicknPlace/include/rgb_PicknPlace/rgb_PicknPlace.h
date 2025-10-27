#pragma once
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core/types.hpp>
#include <memory>
#include <map>
#include <string>
namespace cv
{
class Mat;
}
namespace rgb_picknplace
{
struct PoseRPY
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};
struct PlacementSite
{
  PoseRPY approach;
  PoseRPY place;
};
class RgbPicknPlaceNode
{
public:
  /**
   * @brief Construct the node and wire up ROS interfaces.
   *
   * @param[in] nh Public (global) node handle used for topics and services in the default namespace.
   * @param[in] pnh Private node handle used to resolve parameters under the node's private namespace.
   */
  RgbPicknPlaceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
private:
  /**
   * @brief Load runtime configuration for poses, colour classification, and safety settings.
   */
  void loadParameters();
  /**
   * @brief Retrieve a pose parameter by name, falling back when it is not provided.
   *
   * @param[in] name Parameter key relative to the private namespace.
   * @param[in] fallback Pose to use when the parameter is missing.
   * @return The resolved pose configuration.
   */
  PoseRPY getPoseParam(const std::string& name, const PoseRPY& fallback);
  /**
   * @brief Fetch a placement site definition for a specific colour bucket.
   *
   * @param[in] color Logical colour label (e.g. "red").
   * @param[in] fallback Site definition used when parameters are incomplete.
   * @return Configured placement site.
   */
  PlacementSite getPlacementSiteParam(const std::string& color, const PlacementSite& fallback);
  /**
   * @brief Synchronised callback for colour and depth images from the RealSense camera.
   *
   * @param[in] color_msg Most recent colour frame.
   * @param[in] depth_msg Depth frame paired to the colour image.
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg);
  /**
   * @brief Classify the dominant colour contained in the region of interest.
   *
   * @param[in] roi_bgr ROI patch cropped from the colour image in BGR space.
   * @return Label of the detected colour bucket.
   */
  std::string classifyColor(const cv::Mat& roi_bgr) const;
  /**
   * @brief Compute the region of interest used to sample the detected object.
   *
   * @param[in] image Source image to compute the ROI against.
   * @return Bounding box in image coordinates.
   */
  cv::Rect computeRoi(const cv::Mat& image) const;
  /**
   * @brief Convert the raw depth image values to metres for the grasp height estimate.
   *
   * @param[in] depth_msg Depth image aligned to the colour frame.
   * @param[in] roi Region describing the sampled pixel neighbourhood.
   * @return Estimated pick height in metres.
   */
  double extractDepthMeters(const sensor_msgs::ImageConstPtr& depth_msg, const cv::Rect& roi) const;
  /**
   * @brief Execute the pick-and-place routine for the requested drop site.
   *
   * @param[in] site Placement poses representing the target bin.
   * @param[in] pick_height Height to approach when grasping the object.
   */
  void executePickAndPlace(const PlacementSite& site, double pick_height);
  /**
   * @brief Handle the start service, enabling automatic cycles.
   *
   * @param[in] req Service request (unused).
   * @param[out] resp Response populated with the acknowledgement state.
   * @return True when the service call succeeds.
   */
  bool startService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  /**
   * @brief Handle the stop service, pausing execution after the current motion.
   *
   * @param[in] req Service request (unused).
   * @param[out] resp Response populated with the acknowledgement state.
   * @return True when the service call succeeds.
   */
  bool stopService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  /**
   * @brief Handle the resume service, allowing cycles to continue after a stop.
   *
   * @param[in] req Service request (unused).
   * @param[out] resp Response populated with the acknowledgement state.
   * @return True when the service call succeeds.
   */
  bool resumeService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  /**
   * @brief Handle the emergency-stop service, immediately disabling motion requests.
   *
   * @param[in] req Service request (unused).
   * @param[out] resp Response populated with the acknowledgement state.
   * @return True when the service call succeeds.
   */
  bool estopService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  /**
   * @brief Check whether the node can begin a new pick-and-place cycle.
   *
   * @param[out] reason Optional string that receives the denial reason when not ready.
   * @return True if a new cycle may begin.
   */
  bool canAcceptWork(std::string* reason = nullptr) const;
  /**
   * @brief Enable or disable the automatic processing state machine.
   *
   * @param[in] enabled Desired enabled state.
   */
  void setEnabled(bool enabled);
  /**
   * @brief Command the Dobot to move to a pose specified in roll-pitch-yaw notation.
   *
   * @param[in] pose Target pose the Dobot should reach.
   */
  void moveToPose(const PoseRPY& pose);
  /**
   * @brief Open the gripper using the tool control publisher.
   */
  void openGripper();
  /**
   * @brief Close the gripper around the detected object.
   */
  void closeGripper();
  /**
   * @brief Wait for previously published trajectories to complete.
   */
  void waitForMotion();
  /**
   * @brief Public (global namespace) node handle for topics and services.
   */
  ros::NodeHandle nh_;
  /**
   * @brief Private node handle for parameters scoped under the node's name.
   */
  ros::NodeHandle pnh_;
  
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
  message_filters::Subscriber<sensor_msgs::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Publisher ee_pub_;
  ros::Publisher tool_pub_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
  ros::ServiceServer resume_srv_;
  ros::ServiceServer estop_srv_;
  PoseRPY home_pose_{};
  PoseRPY pre_pick_pose_{};
  PoseRPY pick_pose_{};
  std::map<std::string, PlacementSite> drop_sites_;
  ros::Duration dwell_time_{0.75};
  ros::Duration detection_cooldown_{1.5};
  double min_saturation_{40.0};
  double min_value_{40.0};
  int roi_width_{120};
  int roi_height_{120};
  double depth_scale_{0.001};
  bool adjust_pick_height_{false};
  double pick_surface_offset_{0.0};
  double min_pick_height_{0.0};
  double max_pick_height_{0.10};
  bool busy_;
  bool enabled_{false};
  bool estop_active_{false};
  ros::Time last_detection_time_{};
};
}  // namespace rgb_picknplace
