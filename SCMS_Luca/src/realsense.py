import numpy as np
import pyrealsense2 as rs

class RealSenseRGBD:
    def __init__(self, width=640, height=480, fps=30):
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
        color_stream = self.profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()
        self.intrinsics = {
            "fx": intr.fx, "fy": intr.fy, "cx": intr.ppx, "cy": intr.ppy,
            "width": intr.width, "height": intr.height,
            "depth_scale": self.depth_scale
        }
        for _ in range(10):
            self.pipeline.wait_for_frames()
        self.started = True

    def get_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        d = frames.get_depth_frame(); c = frames.get_color_frame()
        if not d or not c: return None, None
        depth = np.asanyarray(d.get_data())
        color = np.asanyarray(c.get_data())
        return color, depth

    def stop(self):
        if self.started:
            self.pipeline.stop()
            self.started = False
