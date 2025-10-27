#include "rgb_PicknPlace/rgb_PicknPlace.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgb_picknplace");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  rgb_picknplace::RgbPicknPlaceNode node(nh, pnh);

  ros::spin();
  return 0;
}