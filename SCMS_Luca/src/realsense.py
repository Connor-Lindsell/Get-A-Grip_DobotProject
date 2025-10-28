import time
import numpy as np
import pyrealsense2 as rs

class RealSenseRGBD:
    def __init__(self, width = 640, height = 480, fps = 30):
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.align = rs.align(rs.stream.color)
        self.profile = None
        self.depth_scale = None
        self.intrinsics = None
        self.started = False

    def start(self):
        if self.started: return
        self.profile = self.pipeline.start(self.cfg)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        colour_stream = self.profile.get_stream(rs.stream.color)
        intr = colour_stream.as_video_stream_profile().get_intrinsics()
        self.intrinsics = {
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.ppx,
            "cy": intr.ppy,
            "width": intr.width,
            "height": intr.height,
            "depth_scale": self.depth_scale
        }

        for _ in range(10):
            self.pipeline.wait_for_frames()
        self.started = True
        print(f"Camera Started. fx={intr.fx:.1f}, fy={intr.fy:.1f}, cx={intr.ppx:.1f}, cy={intr.ppy:.1f}")

    def get_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        depth = frames.get_depth_frame()
        colour = frames.get_color_frame()
        if not depth or not colour: 
            return None, None
        depth_np = np.asanyarray(depth.get_data())
        colour_np = np.asanyarray(colour.get_data())
        return depth_np, colour_np
    
    def stop(self):
        if self.started:
            self.pipeline.stop()
            self.started = False