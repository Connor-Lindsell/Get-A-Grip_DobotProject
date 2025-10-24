function [p_next, R_next] = ibvsStep(u,v,u0,v0,Z,fx,fy,T_base_cam,T_base_ee,g)
eu = u - u0; ev = v - v0;
dx_cam = -g.kx * eu * Z / fx;
dy_cam = -g.ky * ev * Z / fy;
dz_cam = -g.kz * (Z - g.Zdes);

R_base_cam = T_base_cam(1:3,1:3);
dp_base = R_base_cam * [dx_cam; dy_cam; dz_cam];

p_cur = tform2trvec(T_base_ee).';
R_cur = tform2rotm(T_base_ee);

p_next = p_cur + dp_base;
R_next = R_cur;  % keep tool orientation fixed for top-down pick
end
