"""ROS bridge helper talking to the Dobot course driver over roslibpy."""

import time
import roslibpy

# Safety codes from course driver (INVALID..STOPPED = 0..6)
# We'll mostly care about OPERATING(4). :contentReference[oaicite:2]{index=2}
INVALID, DISCONNECTED, INITIALISING, ESTOPPED, OPERATING, PAUSED, STOPPED = range(7)

class DobotROSClient:
    """
    ROS1-over-WebSocket client using roslibpy.
    Host: IP of the RPi (default 10.42.0.1), Port: 9090 (rosbridge_websocket).
    """
    def __init__(self, host='10.42.0.1', port=9090, namespace='/dobot_magician'):
        self.client = roslibpy.Ros(host=host, port=port)
        self.ns = namespace

        # Publishers
        self.pub_pose = roslibpy.Topic(self.client, f'{self.ns}/target_end_effector_pose', 'geometry_msgs/Pose')
        self.pub_tool_single = roslibpy.Topic(self.client, f'{self.ns}/target_tool_state', 'std_msgs/Int32')
        self.pub_tool_vec = roslibpy.Topic(self.client, f'{self.ns}/target_tool_state', 'std_msgs/Float64MultiArray')
        self.pub_safety = roslibpy.Topic(self.client, f'{self.ns}/target_safety_status', 'std_msgs/Int32')

        # Subscribers
        self.sub_safety = roslibpy.Topic(self.client, f'{self.ns}/safety_status', 'std_msgs/Int32')
        self.sub_ee = roslibpy.Topic(self.client, f'{self.ns}/end_effector_poses', 'geometry_msgs/Pose')

        self._safety = None
        self._ee_pose = None

        self.sub_safety.subscribe(lambda msg: setattr(self, '_safety', int(msg['data'])))
        self.sub_ee.subscribe(lambda msg: setattr(self, '_ee_pose', msg))

    def connect(self, timeout=10.0):
        self.client.run()
        t0 = time.time()
        while not self.client.is_connected and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not self.client.is_connected:
            raise RuntimeError('Failed to connect to rosbridge server.')

    def close(self):
        self.sub_safety.unsubscribe(); self.sub_ee.unsubscribe()
        self.pub_pose.unadvertise(); self.pub_tool_single.unadvertise()
        self.pub_tool_vec.unadvertise(); self.pub_safety.unadvertise()
        self.client.terminate()

    def safety(self):
        return self._safety

    def ee_pose_xyzquat(self):
        """
        Returns (x,y,z, qx,qy,qz,qw) in metres/radians quaternion from /end_effector_poses.
        """
        p = self._ee_pose
        if not p: return None
        pos = p['position']; ori = p['orientation']
        return (pos['x'], pos['y'], pos['z'], ori['x'], ori['y'], ori['z'], ori['w'])

    def ensure_operating(self, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.safety() == OPERATING:
                return True
            time.sleep(0.05)
        return False

    # --- Commands (match course topics) ---
    def set_safety(self, code: int):
        self.pub_safety.publish(roslibpy.Message({'data': int(code)}))

    def send_pose(self, x,y,z, qx,qy,qz,qw):
        msg = {
            'position': {'x': float(x), 'y': float(y), 'z': float(z)},
            'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
        }
        self.pub_pose.publish(roslibpy.Message(msg))

    def suction_onoff(self, on: bool):
        # Single-value variant (0/1) exists in guide. :contentReference[oaicite:3]{index=3}
        self.pub_tool_single.publish(roslibpy.Message({'data': 1 if on else 0}))

    def set_tool_pump_gripper(self, pump_on: bool, gripper_close: bool):
        # Two-element vector [pump, gripper] variant also supported. :contentReference[oaicite:4]{index=4}
        arr = {'data': [1.0 if pump_on else 0.0, 1.0 if gripper_close else 0.0]}
        self.pub_tool_vec.publish(roslibpy.Message(arr))
