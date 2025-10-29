"""Collect RealSense ⇄ robot correspondences using the CHECKERBOARD CENTRE.

Keys:
  c : record checkerboard centre (auto-detected)
  r : record latest mouse click (legacy/manual)
  u : undo last record
  s : solve rigid T_rc and save to config/handeye.json
  q : quit

"""

import json
from pathlib import Path

import cv2
import numpy as np

from realsense import RealSenseRGBD
from robot_io import DobotROSClient


# ===================== User config =====================
# Number of INNER corners (columns, rows). Example: a 7x5 board has 6x4 inner corners.
BOARD_COLS = 20   # inner corners along width (columns)
BOARD_ROWS = 12   # inner corners along height (rows)
SQUARE_SIZE_M = 0.012  # metres (only needed if you later use PnP; we only need the centre pixel here)

# Depth sampling window (pixels) around the selected u,v
DEPTH_PATCH_R = 4  # radius; patch is (2R+1)^2

# Minimum number of points to solve a rigid transform
MIN_POINTS_TO_SOLVE = 3
# ======================================================


def pixel_to_3d(u, v, depth_m_map, intr):
    """Project a pixel (u,v) with depth (metres) into camera-frame XYZ."""
    Z = float(depth_m_map[v, u])
    X = (u - intr["cx"]) / intr["fx"] * Z
    Y = (v - intr["cy"]) / intr["fy"] * Z
    return np.array([X, Y, Z], dtype=float)


def robust_depth_at(depth_m_map, u, v, r=DEPTH_PATCH_R):
    """Median depth (m) in a small window around (u,v). Returns None if no valid depth."""
    h, w = depth_m_map.shape
    u0, v0 = int(round(u)), int(round(v))
    u1, u2 = max(0, u0 - r), min(w, u0 + r + 1)
    v1, v2 = max(0, v0 - r), min(h, v0 + r + 1)
    patch = depth_m_map[v1:v2, u1:u2]
    vals = patch[np.isfinite(patch) & (patch > 0)]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def solve_rigid(Pc, Pr):
    """Rigid transform that maps camera-space points Pc to robot-space points Pr."""
    Pc = np.asarray(Pc); Pr = np.asarray(Pr)
    mc = Pc.mean(axis=0); mr = Pr.mean(axis=0)
    Xc = Pc - mc; Xr = Pr - mr
    U, S, Vt = np.linalg.svd(Xc.T @ Xr)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mr - R @ mc
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def find_checkerboard_center(gray, draw_img=None):
    """
    Detect checkerboard and return subpixel centre (u,v).
    The 'centre' is taken as the mean of all detected corner coordinates.
    """
    pattern_size = (BOARD_COLS, BOARD_ROWS)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ok:
        return None, None
    # Refine to sub-pixel
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    # Optional draw
    if draw_img is not None:
        cv2.drawChessboardCorners(draw_img, pattern_size, corners, ok)
    # Mean of all corners as board centre (u,v)
    uv = corners.reshape(-1, 2).mean(axis=0)
    return float(uv[0]), float(uv[1])


def main():
    # --- Camera ---
    cam = RealSenseRGBD()
    cam.start()
    intr = cam.intrinsics  # dict: fx, fy, cx, cy, width, height, depth_scale

    # --- Robot ---
    bot = DobotROSClient(host="10.42.0.1", port=9090)
    bot.connect()
    bot.ensure_operating(8.0)  # try to reach OPERATING(4), ok if not

    # --- State ---
    clicks = []          # manual clicks (optional)
    Pc_list, Pr_list = [], []  # collected pairs

    # Mouse handler (kept for 'r' legacy)
    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))
            print(f"[calib] click {(x, y)}")

    cv2.namedWindow("calib")
    cv2.setMouseCallback("calib", on_mouse)

    print("[calib] 'c'=record CHECKERBOARD centre  'r'=record last click  'u'=undo  's'=solve  'q'=quit")
    print(f"[calib] Board (inner corners): {BOARD_COLS} x {BOARD_ROWS}  |  Depth patch r={DEPTH_PATCH_R}")

    while True:
        color, depth_raw = cam.get_aligned_frames()
        if color is None:
            continue
        depth_m = depth_raw * intr["depth_scale"]

        vis = color.copy()
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # Try detect checkerboard each frame
        u_cb, v_cb = find_checkerboard_center(gray, draw_img=vis)
        if u_cb is not None:
            cv2.circle(vis, (int(round(u_cb)), int(round(v_cb))), 6, (0, 255, 0), 2)
            cv2.putText(vis, "CB centre", (int(u_cb)+8, int(v_cb)-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(vis, "Checkerboard NOT found", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Draw any manual clicks
        for (x, y) in clicks:
            cv2.circle(vis, (x, y), 4, (0, 255, 255), -1)

        # HUD
        hud = f"[{len(Pc_list)} pts] c=record CB  r=record click  u=undo  s=solve  q=quit"
        cv2.putText(vis, hud, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        cv2.putText(vis, hud, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("calib", vis)
        k = cv2.waitKey(1) & 0xFF

        if k == ord("c"):
            if u_cb is None:
                print("[calib] checkerboard not detected; cannot record.")
                continue
            Zm = robust_depth_at(depth_m, u_cb, v_cb, r=DEPTH_PATCH_R)
            if Zm is None:
                print("[calib] no valid depth at CB centre; try moving slightly.")
                continue
            Pc = pixel_to_3d(int(round(u_cb)), int(round(v_cb)), depth_m, intr)
            p = bot.ee_pose_xyzquat()
            if p is None:
                print("[calib] no EE pose yet (safety not OPERATING?).")
                continue
            x, y, z, *_ = p
            Pr = np.array([x, y, z], dtype=float)
            Pc_list.append(Pc); Pr_list.append(Pr)
            print(f"[calib] recorded CB centre #{len(Pc_list)}  Pc={Pc}  Pr={Pr}")

        elif k == ord("r"):
            if not clicks:
                continue
            u, v = clicks[-1]
            Zm = robust_depth_at(depth_m, u, v, r=DEPTH_PATCH_R)
            if Zm is None:
                print("[calib] no depth at click.")
                continue
            Pc = pixel_to_3d(u, v, depth_m, intr)
            p = bot.ee_pose_xyzquat()
            if p is None:
                print("[calib] no EE pose yet (safety not OPERATING?).")
                continue
            x, y, z, *_ = p
            Pr = np.array([x, y, z], dtype=float)
            Pc_list.append(Pc); Pr_list.append(Pr)
            print(f"[calib] recorded CLICK #{len(Pc_list)}  Pc={Pc}  Pr={Pr}")

        elif k == ord("u"):
            if Pc_list:
                Pc_list.pop(); Pr_list.pop()
                print(f"[calib] undo → {len(Pc_list)} remaining")

        elif k == ord("s"):
            if len(Pc_list) < MIN_POINTS_TO_SOLVE:
                print(f"[calib] need >={MIN_POINTS_TO_SOLVE} points, currently {len(Pc_list)}")
                continue
            T = solve_rigid(Pc_list, Pr_list).tolist()
            Path("config").mkdir(parents=True, exist_ok=True)
            with open("config/handeye.json", "w") as f:
                json.dump(
                    {
                        "version": 1,
                        "frame_camera": "camera_color_optical_frame",
                        "frame_robot": "dobot_base",
                        "T_rc": T,
                        "meta": {
                            "N_points": len(Pc_list),
                            "method": "rigid_points_from_cb_centre",
                            "board_inner_corners": [BOARD_COLS, BOARD_ROWS],
                            "square_size_m": SQUARE_SIZE_M,
                        },
                    },
                    f,
                    indent=2,
                )
            print("[calib] saved config/handeye.json")

        elif k == ord("q"):
            break

    cam.stop()
    cv2.destroyAllWindows()
    bot.close()


if __name__ == "__main__":
    main()
