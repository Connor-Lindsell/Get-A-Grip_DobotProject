% =========================================================================
% IdentifyBlock - Identify a block in the scene based on color and shape
% 
%  This function processes an input image to identify blocks based on their
%  color and shape. It returns the position and orientation of the identified
%  block relative to the base of the robot.
% 
%  Inputs:
%    cameraParams - A structure containing the camera's intrinsic and extrinsic parameters.
%
%  Outputs:
%    blockPosition - A 3x1 vector representing the (x, y, z) position of the block
%                    in the robot's base frame.
%    blockOrientation - A 3x1 vector representing the (roll, pitch, yaw) orientation
%                       of the block in radians.
%    blockColour - A string indicating the color of the identified block.
% =========================================================================

function [blockPosition, blockOrientation, blockColour] = IdentifyBlock(cameraParams)


    disp('=======================================');
    disp('Identifying block in the scene...');

    I = imread('frame0000.jpg'); % For testing without ROS

    % Subscribe to RGB and Depth image topics
    % depthSub = rossubscriber('/camera/depth/image_raw','sensor_msgs/Image');
    % depthMsg = receive(depthSub, 3);
    % depth_img = rosReadImage(depthMsg);  

    % %  Grab one color frame (no rosbag needed)
    % rgbSub = rossubscriber('/camera/color/image_raw','sensor_msgs/Image');
    % rgbMsg = receive(rgbSub, 3);
    % Irgb   = rosReadImage(rgbMsg);  

    % greyscale_img = rgb2gray(rgb_img);

    % For testing without ros
    greyscale_img = rgb2gray(I);

    % detect corner features in the image
    points = detectHarrisFeatures(greyscale_img);
    [features, valid_points] = extractFeatures(greyscale_img, points);

    % Valible points features on figure 
    figure; imshow(greyscale_img); hold on;
    plot(valid_points.selectStrongest(50),'showOrientation',true);

    
    % Get centroid of inlier points
    centroid = mean(inlierPts.Location, 1);
    % Get block color
    blockColour = helperGetBlockColour(rgb_img, round(centroid));
    % Get block position in camera frame
    Pc_cam = getBlockPose3D(centroid, depth_img, cameraParams.Fx, cameraParams.Fy, cameraParams.U0, cameraParams.V0, true);
    % Transform block position to robot base frame
    Pc_base = cameraParams.R_cam2base * Pc_cam + cameraParams.t_cam2base;
    blockPosition = Pc_base;

    % Assume block orientation is aligned with camera frame for simplicity
    blockOrientation = [0; 0; 0]; % Placeholder for actual orientation calculation
end