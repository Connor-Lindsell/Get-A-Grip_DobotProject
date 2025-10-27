# dobot_perception

A minimal ROS 1 (Noetic) perception package scaffold designed to subscribe to an Intel RealSense RGB stream and provide hooks for future block detection and hand-eye calibration against the Dobot Magician.

## Dependencies

The package is set up to use the standard ROS image processing stack:

- [`image_transport`](http://wiki.ros.org/image_transport) for efficient transport of camera frames.
- [`cv_bridge`](http://wiki.ros.org/cv_bridge) to convert `sensor_msgs/Image` messages into OpenCV `cv::Mat` objects.
- [`image_pipeline`](http://wiki.ros.org/image_pipeline) utilities, especially the `camera_calibration` node, to obtain accurate intrinsic parameters from the RealSense camera.
- [`tf2_ros`](http://wiki.ros.org/tf2_ros) to broadcast and listen for transforms once the camera-to-robot extrinsics are known.

Install the binary dependencies via `apt`:

```bash
sudo apt update
sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-image-pipeline ros-noetic-tf2-ros
```

OpenCV is pulled in transitively by `cv_bridge`, but you can also install it explicitly with `sudo apt install ros-noetic-vision-opencv`.

## Building

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun dobot_perception dobot_perception_node
```

Set the `image_topic` private parameter if your RealSense driver publishes a different topic name:

```bash
rosrun dobot_perception dobot_perception_node _image_topic:=/camera/color/image_rect_color
```

## Camera calibration workflow

1. **Intrinsic calibration** – Launch the RealSense camera driver and run the ROS `camera_calibration` tool from the `image_pipeline` package:
   ```bash
   rosrun camera_calibration cameracalibrator.py --size 7x9 --square 0.024 \
     image:=/camera/color/image_raw camera:=/camera/colour/image_raw
   ```
   Replace `--size` and `--square` with the checkerboard dimensions in corners and meters, respectively. Save the generated YAML and load it later as a ROS parameter.

2. **Hand–eye (extrinsic) calibration** – Once the Dobot end effector can move the calibration target, use `industrial_extrinsic_cal` or the `handeye_calibration` package (`ros-noetic-moveit-calibration`). These tools take synchronized robot poses and camera observations to compute the transform between the camera optical frame and the robot base or end effector.

3. **Integrate into the node** – Extend `CameraNode` to read the saved YAML intrinsics via `camera_info_manager` and publish `sensor_msgs/CameraInfo` alongside images. Store the hand–eye transform in the TF tree (e.g., publish `static_transform_publisher`).

This scaffold keeps the focus on perception. After calibration, you can add block detection logic that reads the calibrated RGB-D data and publishes block poses to downstream planning nodes.
