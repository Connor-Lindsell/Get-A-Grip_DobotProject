function Pc_cam = getBlockPose3D(centroid, depthImg, fx, fy, u0, v0, depth_in_mm)
u = round(centroid(1)); v = round(centroid(2));
win = depthImg(max(v-2,1):min(v+2,end), max(u-2,1):min(u+2,end));
Z = median(double(win(:)));
if depth_in_mm, Z = Z/1000.0; end
X = (u - u0) * Z / fx;
Y = (v - v0) * Z / fy;
Pc_cam = [X; Y; Z];
end
