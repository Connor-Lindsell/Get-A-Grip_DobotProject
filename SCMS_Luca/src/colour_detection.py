import cv2
import numpy as np
import pyrealsense2 as rs
import math
import json, os, datetime
from collections import namedtuple

# ----------------- Config -----------------
MIN_AREA = 800
POLY_EPS_FRAC = 0.03
BGR = {'red':(0,0,255),'green':(0,200,0),'blue':(255,0,0),'white':(255,255,255),'black':(0,0,0),'yellow':(0,255,255)}
HSVRange = namedtuple('HSVRange', 'lo hi')
COLOR_RANGES = {
    'red':   [HSVRange((0,110,70),(10,255,255)), HSVRange((170,110,70),(179,255,255))],
    'green': [HSVRange((35,60,60),(85,255,255))],
    'blue':  [HSVRange((90,60,60),(130,255,255))],
}
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

def mask_for_color(hsv_img, color_name):
    mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
    for r in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv_img, np.array(r.lo, np.uint8), np.array(r.hi, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)
    return mask

def find_quads_from_mask(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    quads = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA: continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, POLY_EPS_FRAC*peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            quads.append((approx, area))
    quads.sort(key=lambda x: x[1], reverse=True)
    return [q for q,_ in quads]

def order_quad_points(quad):
    pts = quad.reshape(-1, 2)
    s = pts.sum(axis=1); d = np.diff(pts, axis=1).ravel()
    tl = pts[np.argmin(s)]; br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]; bl = pts[np.argmax(d)]
    return np.array([tl,tr,br,bl], dtype=np.int32)

def median_depth_at_polygon(depth_raw, poly, depth_scale, pad=2):
    h,w = depth_raw.shape
    mask = np.zeros((h,w), np.uint8)
    cv2.fillPoly(mask, [poly], 255)
    if pad>0: mask = cv2.erode(mask, np.ones((pad,pad), np.uint8), 1)
    vals = depth_raw[mask==255]; vals = vals[vals>0]
    if vals.size == 0: return None
    return float(np.median(vals) * depth_scale)

def start_realsense(width=640, height=480, fps=30):
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    s = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = s.get_intrinsics()
    return pipeline, align, depth_scale, (intr.fx, intr.fy, intr.ppx, intr.ppy), (intr.width, intr.height)

def build_snapshot_json(intrinsics, depth_scale, image_size, all_detections):
    fx,fy,cx,cy = intrinsics
    W,H = image_size
    now = datetime.datetime.utcnow().isoformat(timespec='milliseconds') + 'Z'
    snap = {
        "timestamp": now,
        "image_size": {"width": int(W), "height": int(H)},
        "intrinsics": {"fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy)},
        "depth_scale": float(depth_scale),
        "detections": []
    }
    # Flatten detections into the list
    for det in all_detections:
        snap["detections"].append({
            "color": det["color"],
            "centroid": {"u": float(det["centroid"][0]), "v": float(det["centroid"][1])},
            "corners": [[int(p[0]), int(p[1])] for p in det["corners"]],
            "area_px": float(det["area_px"]),
            "depth_m": None if det["depth_m"] is None else float(det["depth_m"]),
        })
    return snap

def save_snapshot(snap, out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    fname = f"detections_{snap['timestamp'].replace(':','-')}.json"
    path = os.path.join(out_dir, fname)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(snap, f, indent=2)
    return path

def main():
    pipeline, align, depth_scale, intr, imsize = start_realsense()
    fx,fy,cx,cy = intr; W,H = imsize
    print(f"[RealSense] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  size={W}x{H}")
    print("[Info] Press 's' to save snapshot JSON; 'h' to print HSV ranges; 'q'/ESC to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            df = frames.get_depth_frame(); cf = frames.get_color_frame()
            if not df or not cf: continue

            depth_raw = np.asanyarray(df.get_data())
            color = np.asanyarray(cf.get_data())
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

            annotated = color.copy()
            detections = []  # will collect for JSON

            for cname in ('red','green','blue'):
                mask = mask_for_color(hsv, cname)
                quads = find_quads_from_mask(mask)
                for quad in quads:
                    area = cv2.contourArea(quad)
                    quad_ord = order_quad_points(quad)
                    # centroid from moments on ordered polygon (or use mean of corners)
                    M = cv2.moments(quad)
                    if M["m00"] > 1e-6:
                        u = M["m10"]/M["m00"]; v = M["m01"]/M["m00"]
                    else:
                        u, v = quad_ord[:,0].mean(), quad_ord[:,1].mean()
                    Z = median_depth_at_polygon(depth_raw, quad_ord, depth_scale, pad=2)

                    # draw
                    cv2.polylines(annotated, [quad_ord], True, BGR[cname], 3, cv2.LINE_AA)
                    for p in quad_ord: cv2.circle(annotated, tuple(p), 5, BGR['yellow'], -1)
                    label = f"{cname.upper()}" + (f"  {Z:.3f} m" if Z is not None else "")
                    tl = tuple(quad_ord[0]); 
                    cv2.rectangle(annotated, (tl[0], tl[1]-24), (tl[0]+180, tl[1]-4), BGR[cname], -1)
                    cv2.putText(annotated, label, (tl[0]+4, tl[1]-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, BGR['white'], 2, cv2.LINE_AA)

                    detections.append({
                        "color": cname,
                        "centroid": (u, v),
                        "corners": quad_ord.tolist(),  # [[u,v]x4]
                        "area_px": area,
                        "depth_m": Z
                    })

            cv2.imshow("Color Blocks (RealSense)", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
            if key == ord('h'):
                print("\n[HSV ranges]")
                for k, ranges in COLOR_RANGES.items():
                    print(f"  {k}: " + " OR ".join([f"[{r.lo} .. {r.hi}]" for r in ranges]))
            if key == ord('s'):
                snap = build_snapshot_json(intr, depth_scale, imsize, detections)
                path = save_snapshot(snap, out_dir=".")
                print(f"[Saved] {path}")

    finally:
        try: pipeline.stop()
        except Exception: pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
