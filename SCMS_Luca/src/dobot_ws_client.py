# dobot_ws_control.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dobot Magician movement + tool test via roslibpy (ROS over WebSocket)

- Connects to Raspberry Pi running rosbridge_websocket
- Publishes joint trajectory and end-effector pose
- Reads current joint state (subscribe + wait)
- Tool control: on+open, on+close, off (Int32MultiArray [pumpOnOff, gripperOpenClose])
- Safety commands (init / E-Stop)

Run:
    python dobot_ws_control.py
"""

import math
import time
from typing import List, Optional, Dict

import roslibpy

# -------------------- Config --------------------
ROSBRIDGE_HOST = '10.42.0.1'  # Pi's IP on hotspot
ROSBRIDGE_PORT = 9090
NS = '/dobot_magician'        # node namespace

# -------------------- Small utils --------------------
def eul2quat(roll, pitch, yaw):
    """ZYX RPY -> [w, x, y, z]"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return [w, x, y, z]

def _wait_for_one(client: roslibpy.Ros, topic_name: str, msg_type: str, timeout: float = 3.0):
    """Subscribe to a topic and wait for one message (dict) or None on timeout."""
    topic = roslibpy.Topic(client, topic_name, msg_type)
    received = {'msg': None}
    def _cb(m): received['msg'] = m
    topic.subscribe(_cb)
    t0 = time.time()
    try:
        while time.time() - t0 < timeout and received['msg'] is None:
            time.sleep(0.02)
    finally:
        try:
            topic.unsubscribe()
            topic.unadvertise()
        except Exception:
            pass
    return received['msg']

# -------------------- Client class --------------------
class DobotWSClient:
    def __init__(self, host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT, ns=NS):
        self.ns = ns
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()
        if not self.client.is_connected:
            raise RuntimeError(f'Could not connect to rosbridge at {host}:{port}')

        # Publishers
        self.pub_joint = roslibpy.Topic(
            self.client, f'{ns}/target_joint_states', 'trajectory_msgs/JointTrajectory')
        self.pub_pose = roslibpy.Topic(
            self.client, f'{ns}/target_end_effector_pose', 'geometry_msgs/Pose')
        self.pub_safety = roslibpy.Topic(
            self.client, f'{ns}/target_safety_status', 'std_msgs/Int32')
        self.pub_tool = roslibpy.Topic(
            self.client, f'{ns}/target_tool_state', 'std_msgs/Int32MultiArray')

    # -------- Movement --------
    def move_joints(self, positions: List[float], duration_sec: float = 1.0,
                    joint_names: Optional[List[str]] = None):
        """Publish a single-point JointTrajectory."""
        if joint_names is None:
            joint_names = []  # fill only if your controller requires names
        msg = {
            'joint_names': joint_names,
            'points': [{
                'positions': list(map(float, positions)),
                'time_from_start': {'secs': int(duration_sec), 'nsecs': int((duration_sec % 1)*1e9)}
            }]
        }
        self.pub_joint.publish(roslibpy.Message(msg))

    def move_ee_pose(self, xyz, rpy):
        """Publish a geometry_msgs/Pose to the EE target topic."""
        w, qx, qy, qz = eul2quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        pose = {
            'position': {'x': float(xyz[0]), 'y': float(xyz[1]), 'z': float(xyz[2])},
            'orientation': {'w': w, 'x': qx, 'y': qy, 'z': qz}
        }
        self.pub_pose.publish(roslibpy.Message(pose))

    # -------- Safety --------
    def estop(self):
        """3 = ESTOPPED"""
        self.pub_safety.publish(roslibpy.Message({'data': 3}))

    def initialise(self):
        """2 = INITIALISING (homes the robot); wait for OPERATING (4) after."""
        self.pub_safety.publish(roslibpy.Message({'data': 2}))

    def get_safety_status(self, timeout: float = 2.0) -> Optional[int]:
        msg = _wait_for_one(self.client, f'{self.ns}/safety_status', 'std_msgs/Int32', timeout)
        return int(msg['data']) if msg and 'data' in msg else None

    # -------- Joint state --------
    def get_joint_state(self, timeout: float = 2.0) -> Dict:
        """
        Returns dict: {name, position, velocity, effort} (lists).
        Empty dict if timeout/no message.
        """
        msg = _wait_for_one(self.client, f'{self.ns}/joint_states', 'sensor_msgs/JointState', timeout)
        if not msg:
            return {}
        return {
            'name': list(msg.get('name', [])),
            'position': list(msg.get('position', [])),
            'velocity': list(msg.get('velocity', [])),
            'effort': list(msg.get('effort', [])),
        }

    def get_joint_positions(self, timeout: float = 2.0) -> List[float]:
        js = self.get_joint_state(timeout)
        return js.get('position', []) if js else []

    # -------- Tool control --------
    def tool_on_open(self):
        """pump=1, gripper=open=1"""
        self.pub_tool.publish(roslibpy.Message({'data': [1, 0]}))

    def tool_on_close(self):
        """pump=1, gripper=close=0 (matches your original Int32MultiArray layout)"""
        self.pub_tool.publish(roslibpy.Message({'data': [1, 1]}))

    def tool_off(self):
        """pump=0, gripper=open=0 (release)"""
        self.pub_tool.publish(roslibpy.Message({'data': [0, 0]}))

    # -------- Cleanup --------
    def close(self):
        try:
            self.pub_joint.unadvertise()
            self.pub_pose.unadvertise()
            self.pub_safety.unadvertise()
            self.pub_tool.unadvertise()
        except Exception:
            pass
        self.client.terminate()

# -------------------- Demo sequence --------------------
def demo_sequence():
    print('Connecting to Dobot via rosbridge...')
    bot = DobotWSClient()

    # Query safety
    st = bot.get_safety_status(timeout=3.0)
    print('Current safety status:', st)

    # Home (INITIALISING=2), then wait for OPERATING(4)
    print('Homing...')
    bot.initialise()
    ok = False
    t0 = time.time()
    while time.time() - t0 < 12.0:
        st = bot.get_safety_status(timeout=1.0)
        if st == 4:
            ok = True
            break
    print('Operating:', ok)

    # Basic joint move
    bot.move_joints([0.0, 0.4, 0.3, 0.0], duration_sec=1.0)
    time.sleep(0.8)

    # Basic EE pose move
    bot.move_ee_pose([0.20, 0.00, 0.10], [0.0, 0.0, 0.0])
    time.sleep(0.8)

    # Tool tests
    bot.tool_on_open();  time.sleep(0.5)
    bot.tool_on_close(); time.sleep(0.5)
    bot.tool_off();      time.sleep(0.3)

    # Sweep second joint
    for k in range(19):  # 0.1:0.05:1.0
        j2 = round(0.1 + 0.05 * k, 3)
        bot.move_joints([0.0, j2, 0.3, 0.0], duration_sec=0.2)
        time.sleep(0.10)

    # Read current joints
    joints = bot.get_joint_positions(timeout=2.0)
    print('Current joints:', joints)

    # Optional safety pulse
    bot.estop();      time.sleep(0.6)
    bot.initialise(); time.sleep(1.2)

    # Final move
    bot.move_joints([0.0, 0.3, 0.2, 0.0], duration_sec=0.8)
    time.sleep(0.8)

    # Full joint state
    js = bot.get_joint_state(timeout=2.0)
    print('Full joint state:', js)

    bot.close()
    print('Demo complete.')

# -------------------- Entry --------------------
if __name__ == '__main__':
    try:
        demo_sequence()
    except KeyboardInterrupt:
        print('Interrupted.')
