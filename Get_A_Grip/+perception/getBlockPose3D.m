% =========================================================
% GETBLOCKPOSE3D: Compute the 3D position of a block from 
%   its centroid in the image
% 
% This function calculates the 3D coordinates of a block 
%   in the camera frame using its 2D centroid in the image
%   and the corresponding depth information.
% 
% Inputs:
%   centroid - A 1x2 vector [u, v] representing the pixel coordinates of the block's centroid in the image.
%   depthImg - A 2D matrix representing the depth image where each pixel value corresponds to depth.
%   fx, fy   - The focal lengths of the camera in pixels.   
%   u0, v0   - The principal point coordinates of the camera in pixels.
%   depth_in_mm - A boolean flag indicating if the depth values are in millimeters (true) or meters (false).
% 
% Outputs:
%   Pc_cam   - A 3x1 vector representing the (X, Y, Z) coordinates of the block in the camera frame.
% =========================================================

function Pc_cam = getBlockPose3D(centroid, depthImg, fx, fy, u0, v0, depth_in_mm)

    u = round(centroid(1)); 
    v = round(centroid(2));

    win = depthImg(max(v-2,1):min(v+2,end), max(u-2,1):min(u+2,end));

    Z = median(double(win(:)));

    if depth_in_mm, Z = Z/1000.0; end

    X = (u - u0) * Z / fx;
    Y = (v - v0) * Z / fy;

    Pc_cam = [X; Y; Z];

    end
