# rgb_PicknPlace

Example ROS (ROS1) package that demonstrates how to drive the Dobot Magician
via the `dobot_magician_driver` topics, classify blocks from an Intel®
RealSense™ RGB-D camera, and route them to colour-specific drop locations.

## What the node does
- Subscribes to synchronised colour and depth images from the RealSense
  (defaults: `/camera/color/image_raw` + `/camera/depth/image_rect_raw`).
- Classifies the average colour within a configurable region-of-interest.
- Optionally uses the depth data to adapt the pick height for the detected
  object.
- Sequences the Dobot Magician through a configurable pick, transport, and
  place routine, including gripper (vacuum) control.
- Exposes simple services so an external supervisor can `start`, `stop`,
  `resume`, or trigger an `estop`.

## Topics
- **Subscriptions**
  - `~color_topic` (`sensor_msgs/Image`, default `/camera/color/image_raw`):
    RealSense RGB frames.
  - `~depth_topic` (`sensor_msgs/Image`, default `/camera/depth/image_rect_raw`):
    RealSense depth frames, 16UC1 (millimetres) or 32FC1 (metres).
- **Publications**
  - `/dobot_magician/target_end_effector_pose` (`geometry_msgs/Pose`)
  - `/dobot_magician/target_tool_state` (`std_msgs/Int32MultiArray`)
- **Services (private namespace)**
  - `~start` (`std_srvs/Trigger`): enable automatic classification.
  - `~stop` (`std_srvs/Trigger`): pause after the current cycle finishes.
  - `~resume` (`std_srvs/Trigger`): clear an estop (if any) and enable again.
  - `~estop` (`std_srvs/Trigger`): latch the node in a safe, stopped state.

## Parameters (private namespace)
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `auto_start` | bool | `false` | Begin processing frames immediately on start-up. |
| `color_topic` | string | `/camera/color/image_raw` | RGB topic to subscribe to. |
| `depth_topic` | string | `/camera/depth/image_rect_raw` | Depth topic to subscribe to. |
| `sync_queue` | int | `5` | Queue size for colour/depth synchronisation. |
| `detection_cooldown` | double | `1.5` | Minimum seconds between pick cycles. |
| `roi_width`, `roi_height` | int | `120` | Size (pixels) of the ROI centred in the image. |
| `min_saturation`, `min_value` | double | `40` | HSV thresholds for a valid colour detection. |
| `adjust_pick_height` | bool | `false` | Use depth data to adapt the pick height. |
| `depth_scale` | double | `0.001` | Conversion factor for 16UC1 depth (RealSense default mm→m). |
| `pick_surface_offset` | double | `0.0` | Metres to subtract from measured depth for tool offset. |
| `min_pick_height`, `max_pick_height` | double | `0.0`, `0.10` | Clamp bounds when using adaptive height. |
| `dwell_time` | double | `0.75` | Seconds to wait after each motion command. |
| `home_pose/*` | double | `0.2, 0, 0.15, 0, 0, 0` | Home pose RPY (metres/radians). |
| `pre_pick_pose/*` | double | `0.25, 0, 0.10, 0, 0, 0` | Hover pose above pick site. |
| `pick_pose/*` | double | `0.25, 0, 0.025, 0, 0, 0` | Baseline pick pose (overridden if adaptive). |
| `{colour}_approach/*` | double | varies | Approach pose for each colour drop site. |
| `{colour}_place/*` | double | varies | Final place pose for each colour. |

## Example launch
```xml
<launch>
  <node pkg="rgb_PicknPlace" type="rgb_picknplace_node" name="color_sort" output="screen">
    <param name="auto_start" value="true" />
    <param name="adjust_pick_height" value="true" />
    <param name="pick_surface_offset" value="0.035" />
    <param name="red_place/x" value="0.12" />
  </node>
</launch>
```

To manually exercise the services:
```bash
rosservice call /color_sort/start
rosservice call /color_sort/estop
rosservice call /color_sort/resume
```

Tune the poses and HSV thresholds for your workcell—the defaults are only
coarse placeholders.