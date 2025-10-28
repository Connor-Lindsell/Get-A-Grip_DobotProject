import json, time, numpy as np
from transformations import quaternion_from_euler
from realsense import RealSenseRGBD
from ibvs_control import ibvs_twist, adjoint
from robot_io import DobotROSClient, OPERATING

def clamp(v, lo, hi): return max(lo, min(hi, v))

def integrate_pose(nominal, v_r, dt=0.04, v_lin_cap=0.04, v_ang_cap=0.3):
    vx,vy,vz, wx,wy,wz = v_r
    vx = clamp(vx, -v_lin_cap, v_lin_cap)
    vy = clamp(vy, -v_lin_cap, v_lin_cap)
    vz = clamp(vz, -v_lin_cap, v_lin_cap)
    wz = clamp(wz, -v_ang_cap, v_ang_cap)
    x,y,z,yaw = nominal
    x += vx*dt; y += vy*dt; z += vz*dt; yaw += wz*dt
    return (x,y,z,yaw)

def main():
    # Load assets
    T_rc = np.array(json.load(open("config/handeye.json"))["T_rc"])
    Ad_rc = adjoint(T_rc)
    blocks = json.load(open("detections/blocks_snapshot.json"))["blocks"]

    # Camera
    cam = RealSenseRGBD(); cam.start(); intr = cam.intrinsics
    u_star, v_star = intr["cx"], intr["cy"]

    # Robot via roslibpy
    bot = DobotROSClient(host='10.42.0.1', port=9090); bot.connect()
    if not bot.ensure_operating(8.0):
        # You can trigger INITIALISING(2) then wait for OPERATING(4) per your guide. :contentReference[oaicite:5]{index=5}
        print("[main] Robot not OPERATING(4). Please home/initialise on the RPi, then re-run.")
        cam.stop(); bot.close(); return

    # Nominal start (metres & radians)
    nominal = [0.20, 0.00, 0.12, 0.0]
    q = quaternion_from_euler(0,0,nominal[3])
    bot.send_pose(nominal[0], nominal[1], nominal[2], q[0], q[1], q[2], q[3])

    for b in blocks:
        label = b.get("label","block"); u,v = b["centroid_px"]; Zsnap = b["mean_depth_m"]
        print(f"[main] Target id={b['id']} label={label}")

        settled = 0
        while True:
            if bot.safety() != OPERATING:
                time.sleep(0.05); continue
            color, depth = cam.get_aligned_frames()
            if color is None: continue
            depth_m = depth * intr["depth_scale"]
            Z = float(depth_m[min(max(v,0),intr["height"]-1), min(max(u,0),intr["width"]-1)])
            if Z <= 0: Z = Zsnap

            Z_target = max(Z + 0.04, 0.02)
            v_c = ibvs_twist(u, v, Z, u_star, v_star, intr["fx"], intr["fy"], intr["cx"], intr["cy"], lam=0.35)
            v_c[2] += 0.6 * (Z_target - Z)
            v_r = Ad_rc @ v_c

            nominal = integrate_pose(nominal, v_r, dt=0.04)
            q = quaternion_from_euler(0,0,nominal[3])
            bot.send_pose(nominal[0], nominal[1], nominal[2], q[0], q[1], q[2], q[3])

            if abs(u-u_star) < 5 and abs(v-v_star) < 5 and abs(Z - Z_target) < 0.01:
                settled += 1
            else:
                settled = 0
            if settled > 5: break
            time.sleep(0.04)

        # descend → grip → lift → place
        bot.send_pose(nominal[0], nominal[1], nominal[2]-0.02, q[0], q[1], q[2], q[3]); time.sleep(1.0)
        bot.set_tool_pump_gripper(True, True); time.sleep(0.5)
        bot.send_pose(nominal[0], nominal[1], nominal[2]+0.03, q[0], q[1], q[2], q[3]); time.sleep(0.8)

        drop_y = {"red": -0.10, "blue": 0.10, "green": 0.00}.get(label, 0.05)
        bot.send_pose(0.22, drop_y, nominal[2], q[0], q[1], q[2], q[3]); time.sleep(0.8)
        bot.set_tool_pump_gripper(False, False); time.sleep(0.4)

    cam.stop(); bot.close(); print("[main] Done.")

if __name__ == "__main__":
    main()
