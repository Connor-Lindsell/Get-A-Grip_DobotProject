import os, json, time, argparse
import numpy as np, cv2 
from pathlib import Path
from realsense import RealSenseRGBD
import pyrealsense2 as rs

def snapshot(blocks, out_path):
    snap = {
        "version": 1,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "frame_id": "camera_colour_optical_frame",
        "blocks": blocks
    }
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(snap, f, indent=2)
    print(f"[detect_blocks] Saved: {out_path}")

def masks(colour, depth_m, zmin, zmax, hsv_ranges):
    mask_depth = (depth_m >= zmin) & (depth_m <= zmax)
    hsv = cv2.cvtColor(colour, cv2.COLOR_BGR2HSV)
    mask_total = np.zeros(mask_depth.shape, np.uint8)
    for (lo, hi) in hsv_ranges:
        m = cv2.inRange(hsv, lo, hi)
        mask_total = cv2.bitwise_or(mask_total, m)
    mask = (mask_total > 0) & mask_depth
    mask = (mask.astype(np.uint8) * 255)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    return mask
    
def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="detections/blocks_snapshot.json")
    ap.add_argument("--zmin", type=float, default=0.18)
    ap.add_argument("--zmax", type=float, default=0.35)
    args = ap.parse_args()

    cam = RealSenseRGBD()
    cam.start()
    intr = cam.intrinsics 
