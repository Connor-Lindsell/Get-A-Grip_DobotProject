import numpy as np

def ibvs_twist(u, v, Z, u_star, v_star, fx, fy, cx, cy, lam=0.35):
    if Z <= 1e-4:
        return np.zeros(6)
    du = float(u - u_star); dv = float(v - v_star)
    L = np.array([
        [-fx/Z,   0.0,   (u-cx)/Z,   ((u-cx)*(v-cy))/fy, -(fx**2+(u-cx)**2)/fx,  (v-cy)],
        [  0.0,  -fy/Z,  (v-cy)/Z,    (fy**2+(v-cy)**2)/fy, -((u-cx)*(v-cy))/fx, -(u-cx)]
    ], dtype=float)
    e = np.array([du, dv], dtype=float)
    v_c = -lam * np.linalg.pinv(L, rcond=1e-3) @ e
    return v_c  # [vx,vy,vz,wx,wy,wz] camera frame

def adjoint(T):
    R = T[:3,:3]; p = T[:3,3]
    px = np.array([[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]])
    return np.block([[R, np.zeros((3,3))],[px@R, R]])
