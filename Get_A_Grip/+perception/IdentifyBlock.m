% =========================================================================
% IdentifyBlock - Identify a block in the scene based on color and shape
% 
%  This function processes an input image to identify blocks based on their
%  color and shape. It returns the position and orientation of the identified
%  block relative to the base of the robot.
% 
%  Inputs:
%    rosbagFile - The path to the ROS bag file containing the image data.
%    cameraParams - A structure containing the camera's intrinsic and extrinsic parameters.
%
%  Outputs:
%    blockPosition - A 3x1 vector representing the (x, y, z) position of the block
%                    in the robot's base frame.
%    blockOrientation - A 3x1 vector representing the (roll, pitch, yaw) orientation
%                       of the block in radians.
%    blockColour - A string indicating the color of the identified block.
% =========================================================================

function [blockPosition, blockOrientation, blockColour] = IdentifyBlock(rosbagFile, cameraParams)

    bag = rosbag(rosbagFile);

    % read first depth image message
    depth_img = select(bag,'Topic', '/camera/depth/image_rect_raw');
    depth_img = readMessages(depth_img, 1);
    depth_img = readImage(depth_img{1});

    % read first rgb image message
    rgb_img = select(bag,'Topic', '/camera/color/image_raw/compressed');
    rgb_img = readMessages(rgb_img, 5);    
    rgb_img = readImage(rgb_img{1});
    rgb_img = imrotate(rgb_img, -90);

    greyscale_img = rgb2gray(rgb_img);

    pts = detectSIFTFeatures(greyscale_img, 'MetricThreshold', 500);
    [features, pts] = extractFeatures(greyscale_img, pts);

    % inliers after RANSAC
    [~, inlierIdx] = helperRANSAC(pts.Location, features);
    inlierPts = pts(inlierIdx, :);
    if isempty(inlierPts)
        error('No block identified in the scene');
    end

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