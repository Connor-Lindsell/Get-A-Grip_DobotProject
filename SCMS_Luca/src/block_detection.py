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
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="detections/blocks_snapshot.json")
    ap.add_argument("--zmin", type=float, default=0.18)
    ap.add_argument("--zmax", type=float, default=0.35)
    args = ap.parse_args()

    intr = cam.intrinsics 
    cam = RealSenseRGBD()
    cam.start()
    print("detect blocks, press +/- for z, c for colour, s to save, q to quit")

    colours = {
        "red":  [((0,120,70),(10,255,255)), ((170,120,70),(180,255,255))],
        "blue": [((100,120,70),(130,255,255))],
        "green":[((40, 80,70),(80, 255,255))]
    }

    labels = list(colours.keys())
    label_idx = 0
    zmin, zmax = args.zmin, args.zmax

    while True:
        colour, depth = cam.get_aligned_frames()
        if colour is None: continue
        depth_m = depth * intr["depth_scale"]

        hsv_ranges = colours[labels[label_idx]]
        mask = masks(colour, depth_m, zmin, zmax, hsv_ranges)
        vis = colour.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blocks = []
        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            if area < 400: continue
            rect = cv2.minAreaRect(c)
            (uc, vc), (w, h), _ = rect
            rect_area = max(w*h, 1.0)
            rectangularity = float(area/rect_area)
            if rectangularity < 0.85: continue

            M = cv2.moments(c)
            if M["m00"] == 0: continue
            u = int(M["m10"]/M["m00"]); v = int(M["m01"]/M["m00"])

            mask_c = np.zeros_like(mask); cv2.drawContours(mask_c, [c], -1, 255, -1)
            z_vals = depth_m[mask_c > 0]
            if z_vals.size == 0: continue
            mean_z = float(np.median(z_vals))

            cv2.circle(vis, (u,v), 4, (0,255,0), -1)
            cv2.putText(vis, f"{labels[label_idx]} z={mean_z:.3f}m", (u+6,v-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)

            blocks.append({
                "id": i,
                "label": labels[label_idx],
                "centroid_px": [int(u), int(v)],
                "mean_depth_m": mean_z,
                "rectangularity": rectangularity
            })

        cv2.putText(vis, f"zmin={zmin:.2f} zmax={zmax:.2f} label={labels[label_idx]}    [ +/- ][ c ][ s ][ q ]",
                    (8,24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
        cv2.putText(vis, f"zmin={zmin:.2f} zmax={zmax:.2f} label={labels[label_idx]}    [ +/- ][ c ][ s ][ q ]",
                    (8,24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        cv2.imshow("detect_blocks", vis)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('+'): zmax += 0.01
        elif k == ord('-'): zmin = max(0.0, zmin - 0.01)
        elif k == ord('c'): label_idx = (label_idx + 1) % len(labels)
        elif k == ord('s'):
            snapshot(blocks, args.out)
        elif k == ord('q'):
            break

    cam.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

    
