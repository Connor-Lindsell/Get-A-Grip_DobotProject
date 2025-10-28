import json, time
import numpy as np, cv2
from pathlib import Path
from realsense import RealSenseRGBD
from robot_io import DobotROSClient, OPERATING

def pixel_to_3d(u, v, depth_m, intr):
    Z = float(depth_m[v, u])
    X = (u - intr["cx"])/intr["fx"] * Z
    Y = (v - intr["cy"])/intr["fy"] * Z
    return np.array([X, Y, Z], dtype=float)

def solve_rigid(Pc, Pr):
    Pc = np.asarray(Pc); Pr = np.asarray(Pr)
    mc = Pc.mean(axis=0); mr = Pr.mean(axis=0)
    Xc = Pc - mc; Xr = Pr - mr
    U,S,Vt = np.linalg.svd(Xc.T @ Xr)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0: Vt[-1,:]*=-1; R = Vt.T @ U.T
    t = mr - R @ mc
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=t
    return T

def main():
    cam = RealSenseRGBD(); cam.start(); intr = cam.intrinsics
    bot = DobotROSClient(host='10.42.0.1', port=9090); bot.connect()

    print("[calib] Ensure driver is OPERATING(4). If not, home the robot per your lab guide.")
    if not bot.ensure_operating(8.0):
        print("[calib] Warning: safety != OPERATING(4). Proceeding anyway (pose may not update).")

    clicks, Pc_list, Pr_list = [], [], []

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x,y)); print(f"[calib] click {x,y}")

    cv2.namedWindow("calib"); cv2.setMouseCallback("calib", on_mouse)
    print("[calib] Move EE tip to a visible dot; click it; press 'r' to record. After ≥5, 's' to save. 'q' to quit.")

    while True:
        color, depth = cam.get_aligned_frames()
        if color is None: continue
        depth_m = depth * intr["depth_scale"]
        vis = color.copy()
        for (x,y) in clicks: cv2.circle(vis, (x,y), 4, (0,255,0), -1)
        cv2.putText(vis, f"[{len(Pc_list)} pts] r=record u=undo s=solve q=quit",
                    (8,24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
        cv2.putText(vis, f"[{len(Pc_list)} pts] r=record u=undo s=solve q=quit",
                    (8,24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
        cv2.imshow("calib", vis)
        k = cv2.waitKey(1) & 0xFF

        if k == ord('r'):
            if not clicks: continue
            u,v = clicks[-1]
            Pc = pixel_to_3d(u, v, depth_m, intr)
            p = bot.ee_pose_xyzquat()
            if p is None:
                print("[calib] no EE pose yet — is the driver OPERATING(4)?"); continue
            x,y,z, *_ = p
            Pr = np.array([x,y,z], dtype=float)  # already metres from driver docs
            Pc_list.append(Pc); Pr_list.append(Pr)
            print(f"[calib] recorded #{len(Pc_list)}")
        elif k == ord('u'):
            if Pc_list: Pc_list.pop(); Pr_list.pop()
        elif k == ord('s'):
            if len(Pc_list) < 3:
                print("[calib] need ≥3 points"); continue
            T = solve_rigid(Pc_list, Pr_list).tolist()
            Path("config").mkdir(parents=True, exist_ok=True)
            with open("config/handeye.json","w") as f:
                json.dump({"version":1,"frame_camera":"camera_color_optical_frame","frame_robot":"dobot_base","T_rc":T}, f, indent=2)
            print("[calib] saved config/handeye.json")
        elif k == ord('q'):
            break

    cam.stop(); cv2.destroyAllWindows(); bot.close()

if __name__ == "__main__":
    main()
