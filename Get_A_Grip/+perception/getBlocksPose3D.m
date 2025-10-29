% =========================================================
% GETBLOCKsPOSE3D-2.0: 
% 
% Gets the number of blocks and their positions and orientations
% 
% - first we identify the number of blocks and the corner values identified
% in identifyBlocks func
% 
% we then tranform these corner pixel values and depth into a position
% reference to the camera and from there we can tranform this to a position
% reference to the robot. this happens for all 
% 
% 
%   camParam.intr      : cameraIntrinsics object
%   camParam.K         : 3x3 intrinsic matrix (Computer Vision Toolbox convention)
%   camParam.kc        : [k1 k2 p1 p2 k3] Bouguet order
%   camParam.radial    : [k1 k2 k3]
%   camParam.tangential: [p1 p2]
%   camParam.imageSize : [ny nx]
%   camParam.focal     : [fx fy]
%   camParam.principal : [cx cy]
%   camParam.skew      : alpha_c
% 
% from other files, i need outputs that include the transform to robot from camera (this needs to be a matrix of size 4x4)
% the size of the block (edge length in metres),
% and the blocks struct from identify blocks (matrix of size 4x2 for each block)
% =========================================================

function blocksPose = getBlocksPose3D(T_robot_from_cam, blockSize, blocks)

    import perception.*

    % sources T_robot_from_cam from getCameraCalibration.m, or provides 4x4 identity matrix
    if nargin < 1 || isempty(T_robot_from_cam), T_robot_from_cam = eye(4); end
    assert(nargin >= 2 && isscalar(blockSize) && blockSize > 0, ...
        'Provide blockSize (edge length in metres).');

    camParam = getCameraCalibration;
    intr = camParam.intr;

    % sources 'blocks' from identifyBlocks.m 
    if nargin < 3 || isempty(blocks)
        assert(exist('identifyBlocks','file') == 2, ...
            'identifyBlocks.m not found and no blocks provided.');
        blocks = identifyBlocks();
    end
    if isempty(blocks)
        blocksPose = struct([]); return;
    end

    % creates 3D model of block in its own frame (Z=0)
    h = blockSize/2;
    modelXY = [ -h, -h;
                 h, -h;
                 h,  h;
                -h,  h ]';
    model3D = [modelXY; zeroes(4,1)];

    % for each block, solve perspective and point to get its pose
    n = numel(blocks);
    blocksPose = repmat(struct(),1,n);
    for i = 1:n
        uv = blocks(i).corners;
        assert(isequal(size(uv),[4 2]), ...
            'Block %d corners should be a 4x2 array of pixel coordinates.', i);

            [R_cw, t_cw] = extrinsics(uv, modelXY, intr);  % world = block frame on Z=0
            T_cam_from_block = rt2T(R_cw, t_cw); % transform from block frame to camera frame
            T_robot_from_block = T_robot_from_cam * T_cam_from_block; % transform from block frame to robot frame

        corners_robot = (T_robot_from_block * [model3D ones(4,1)]').'; % 4x4 -> 4x3 ??
        corners_robot = corners_robot(:,1:3); % get rid of homogeneous coord
        centroid_robot = mean(corners_robot,1); % 1x3
        q_robot = rotm2quat(T_robot_from_block(1:3,1:3));  % [w x y z]
        
        blocksPose(i).T_cam_from_block   = T_cam_from_block; % transform from block frame to camera frame
        blocksPose(i).T_robot_from_block = T_robot_from_block; % transform from block frame to robot frame
        blocksPose(i).position_robot     = T_robot_from_block(1:3,4).'; % 1x3
        blocksPose(i).quaternion_robot   = q_robot; % [w x y z]
        blocksPose(i).corners_robot      = corners_robot; % 4x3
        blocksPose(i).centroid_robot     = centroid_robot; % 1x3
        if isfield(blocks,'meta'), blocksPose(i).meta = blocks(i).meta; end
    end

    fprintf('[getBlocksPose3D] %d block(s).\n', n); % display info
end

function T = rt2T(R, t) % build homogeneous transform from R,t
    T = eye(4); T(1:3,1:3) = R; T(1:3,4) = t(:); 
end
