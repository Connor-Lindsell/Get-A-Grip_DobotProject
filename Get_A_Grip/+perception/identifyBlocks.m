% =================================================================
% identifyBlocks.m
% -----------------------------------------------------------------
%   Captures one frame from RealSense camera via ROS, allows user to click,
%   and returns the 3D point in camera coordinates.
%
%   OUTPUT:
%     xyzCamFrame : [x; y; z] point in meters (camera frame)
%     rgbImage    : Undistorted RGB image shown to user
%     depthImage  : Raw depth image in meters
% =================================================================


function [xyzCamFrame, rgbImage, depthImage] = identifyBlocks()

    import perception.*

    % === Load camera intrinsics ===
    camParam = getCameraCalibration();  
    K = camParam.K;                     % Intrinsic matrix
    fx = K(1,1); fy = K(2,2);
    cx = K(1,3); cy = K(2,3);

    % % === Connect to ROS if not already connected ===
    % if ~robotics.ros.internal.Global.isNodeActive
    %     rosinit;
    % end

    % === Subscribe to color and depth topics ===
    colorSub = rossubscriber('/camera/color/image_raw', 'sensor_msgs/Image');
    depthSub = rossubscriber('/camera/depth/image_raw', 'sensor_msgs/Image');

    disp('[INFO] Waiting for color and depth images...');
    colorMsg = receive(colorSub, 5);  % 5 second timeout
    depthMsg = receive(depthSub, 5);

    % === Convert ROS images to MATLAB ===
    rgbRaw     = rosReadImage(colorMsg);
    depthRaw   = double(rosReadImage(depthMsg)) / 1000;  % Convert mm → m

    % === Undistort RGB ===
    rgbImage = undistortImage(rgbRaw, camParam.intr);

    % === Display RGB and get user click ===
    figure; imshow(rgbImage);
    title('Click a point on the image to identify 3D location');
    [u, v] = ginput(1); % u = column (x), v = row (y)
    u = round(u); v = round(v);

    % === Get corresponding depth ===
    depthImage = depthRaw;  % Already converted to meters
    if v > size(depthImage, 1) || u > size(depthImage, 2)
        warning('Clicked point outside image bounds.');
        xyzCamFrame = [NaN; NaN; NaN];
        return;
    end

    Z = depthImage(v, u);  % meters

    if Z == 0 || isnan(Z)
        warning('Invalid depth at selected point.');
        xyzCamFrame = [NaN; NaN; NaN];
        return;
    end

    % === Back-project to 3D (camera frame) ===
    X = (u - cx) * Z / fx;
    Y = (v - cy) * Z / fy;
    xyzCamFrame = [X; Y; Z];

    fprintf('[INFO] 3D Point (camera frame): [%.3f, %.3f, %.3f] meters\n', X, Y, Z);
end
