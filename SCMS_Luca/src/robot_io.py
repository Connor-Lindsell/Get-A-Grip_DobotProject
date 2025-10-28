import time
import roslibpy

# Safety codes used by the Dobot course driver
INVALID, DISCONNECTED, INITIALISING, ESTOPPED, OPERATING, PAUSED, STOPPED = range(7)


class DobotROSClient:
    """
    Dobot ROS client over rosbridge (roslibpy).
    - Connects to ws://<host>:<port> (default host='10.42.0.1', port=9090)
    - Publishes:
        <ns>/target_end_effector_pose           (geometry_msgs/Pose)
        <ns>/target_tool_state                  (std_msgs/Int32)  or (std_msgs/Float64MultiArray [pump,gripper])
        <ns>/target_safety_status               (std_msgs/Int32)
    - Subscribes:
        <ns>/safety_status                      (std_msgs/Int32)
        <ns>/end_effector_pose(s)               (geometry_msgs/Pose or PoseStamped)  ← auto-detected

    Typical use:
        bot = DobotROSClient(host='10.42.0.1', port=9090)
        bot.connect()
        bot.ensure_operating(8.0)
        x,y,z,qx,qy,qz,qw = bot.ee_pose_xyzquat(wait=True, timeout=2.0)
        bot.send_pose(x, y, z, qx, qy, qz, qw)
        bot.set_tool_pump_gripper(True, True)
        bot.close()
    """

    def __init__(self,
                 host: str = '10.42.0.1',
                 port: int = 9090,
                 namespace: str = '/dobot_magician',
                 is_secure: bool = False):
        self.host = host
        self.port = port
        self.ns = namespace.rstrip('/')
        self.client = roslibpy.Ros(host=self.host, port=self.port, is_secure=is_secure)

        # Publishers
        self.pub_pose = roslibpy.Topic(self.client, f'{self.ns}/target_end_effector_pose', 'geometry_msgs/Pose')
        self.pub_tool_i32 = roslibpy.Topic(self.client, f'{self.ns}/target_tool_state', 'std_msgs/Int32')
        self.pub_tool_vec = roslibpy.Topic(self.client, f'{self.ns}/target_tool_state', 'std_msgs/Float64MultiArray')
        self.pub_safety   = roslibpy.Topic(self.client, f'{self.ns}/target_safety_status', 'std_msgs/Int32')

        # Subscribers (safety always known; EE pose resolved dynamically)
        self.sub_safety = roslibpy.Topic(self.client, f'{self.ns}/safety_status', 'std_msgs/Int32')
        self.sub_ee = None  # will be created after connection
        self._ee_topic_name = None

        # State
        self._safety = None
        self._ee_pose = None  # tuple (x,y,z,qx,qy,qz,qw)

        # Hook safety updates
        self.sub_safety.subscribe(lambda msg: setattr(self, '_safety', int(msg['data'])))

    # ------------------- connection lifecycle -------------------

    def connect(self, timeout: float = 10.0):
        self.client.run()
        t0 = time.time()
        while not self.client.is_connected and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not self.client.is_connected:
            raise RuntimeError(f'Failed to connect to rosbridge at {self.host}:{self.port}')

        # Advertise publishers (good hygiene with rosbridge)
        for t in (self.pub_pose, self.pub_tool_i32, self.pub_tool_vec, self.pub_safety):
            t.advertise()

        # Try to subscribe to any valid EE pose topic/type
        if not self._try_subscribe_ee_pose():
            print("[robot_io] Warning: couldn't receive EE pose on expected topics.")
            print("  Run `rostopic list | grep -i end_effector` on the robot and update candidates below if needed.")

    def close(self):
        try:
            if self.sub_ee:
                self.sub_ee.unsubscribe()
            self.sub_safety.unsubscribe()
        except Exception:
            pass
        for t in (self.pub_pose, self.pub_tool_i32, self.pub_tool_vec, self.pub_safety):
            try:
                t.unadvertise()
            except Exception:
                pass
        self.client.terminate()

    # ------------------- helpers -------------------

    def safety(self):
        return self._safety

    def ensure_operating(self, timeout: float = 8.0) -> bool:
        """Wait until safety == OPERATING (4). Returns True if reached."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._safety == OPERATING:
                return True
            time.sleep(0.05)
        return False

    def ee_pose_xyzquat(self, wait: bool = False, timeout: float = 2.0):
        """Return (x,y,z,qx,qy,qz,qw). Optionally wait up to timeout for first message."""
        if wait and self._ee_pose is None:
            t0 = time.time()
            while self._ee_pose is None and time.time() - t0 < timeout:
                time.sleep(0.05)
        return self._ee_pose

    # ------------------- publishers (commands) -------------------

    def set_safety(self, code: int):
        self.pub_safety.publish(roslibpy.Message({'data': int(code)}))

    def send_pose(self, x, y, z, qx, qy, qz, qw):
        self.pub_pose.publish(roslibpy.Message({
            'position':   {'x': float(x), 'y': float(y), 'z': float(z)},
            'orientation':{'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
        }))

    def suction_onoff(self, on: bool):
        """Some course setups use a single Int32 0/1 for the suction pump."""
        self.pub_tool_i32.publish(roslibpy.Message({'data': 1 if on else 0}))

    def set_tool_pump_gripper(self, pump_on: bool, gripper_close: bool):
        """Others expect a 2-element Float64MultiArray: [pump, gripper]."""
        self.pub_tool_vec.publish(roslibpy.Message({
            'data': [1.0 if pump_on else 0.0, 1.0 if gripper_close else 0.0]
        }))

    # ------------------- internal: EE pose subscription -------------------

    def _normalize_pose(self, msg_dict):
        """
        Accept geometry_msgs/Pose  or  geometry_msgs/PoseStamped (dict from roslibpy),
        return (x,y,z,qx,qy,qz,qw) or None if malformed.
        """
        # PoseStamped?
        if 'pose' in msg_dict:
            pos = msg_dict['pose'].get('position', {})
            ori = msg_dict['pose'].get('orientation', {})
        else:
            pos = msg_dict.get('position', {})
            ori = msg_dict.get('orientation', {})

        try:
            return (float(pos['x']), float(pos['y']), float(pos['z']),
                    float(ori['x']), float(ori['y']), float(ori['z']), float(ori['w']))
        except Exception:
            return None

    def _try_subscribe_ee_pose(self) -> bool:
        """
        Try a set of likely topic names and message types. We:
          1) optionally fetch the topic list (requires rosapi; may return None)
          2) attempt subscription anyway (roslibpy allows it)
          3) wait briefly for first message to confirm
        """
        candidates = [
            (f'{self.ns}/end_effector_poses', 'geometry_msgs/PoseStamped'),
            (f'{self.ns}/end_effector_pose',  'geometry_msgs/PoseStamped'),
            (f'{self.ns}/end_effector_poses', 'geometry_msgs/Pose'),
            (f'{self.ns}/end_effector_pose',  'geometry_msgs/Pose'),
        ]

        # Try to get topics (rosapi must be running for this to work)
        available = set()
        try:
            topics = roslibpy.Topic.get_topics(self.client) or []
            available = {t['name'] for t in topics}
        except Exception:
            # rosapi not available; we'll brute-try
            pass

        for name, mtype in candidates:
            if available and name not in available:
                continue  # skip obvious misses if we have a list
            sub = roslibpy.Topic(self.client, name, mtype)

            def _cb(msg, self=self):
                norm = self._normalize_pose(msg)
                if norm is not None:
                    self._ee_pose = norm

            try:
                sub.subscribe(_cb)
                # wait a moment for the first pose
                t0 = time.time()
                while self._ee_pose is None and time.time() - t0 < 2.0:
                    time.sleep(0.05)
                if self._ee_pose is not None:
                    self.sub_ee = sub
                    self._ee_topic_name = name
                    print(f"[robot_io] Subscribed EE pose on '{name}' ({mtype})")
                    return True
                # no message; clean up and try next
                sub.unsubscribe()
            except Exception:
                # subscription failed (wrong type or topic), try the next one
                try:
                    sub.unsubscribe()
                except Exception:
                    pass

        return False


# ------------------- optional: quick CLI smoke test -------------------
if __name__ == "__main__":
    bot = DobotROSClient()
    try:
        bot.connect()
        print("Connected to rosbridge.")
        print("Safety:", bot.safety())
        if bot.ensure_operating(5.0):
            print("Robot is OPERATING (4).")
        else:
            print("Robot not OPERATING within timeout.")

        pose = bot.ee_pose_xyzquat(wait=True, timeout=3.0)
        print("EE pose:", pose)
    finally:
        bot.close()