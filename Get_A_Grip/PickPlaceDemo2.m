function PickPlaceDemo2(jsonFile)
% =========================================================================
% PickPlaceDemo.m (JSON-driven)
% - Reads RealSense detections JSON (u,v,Z and corners)
% - Converts to base-frame pick poses via intrinsics + T_base_cam
% - Builds block structs (colour, blockPosition, hoverPosition, orientation)
% - Runs your existing pick → place logic
% =========================================================================
%
% Usage:
%   PickPlaceDemo('detections_2025-10-29T15-45-12.345Z.json')
%   % or if omitted, a file chooser will open.

    if nargin < 1 || ~isfile(jsonFile)
        [f,p] = uigetfile('*.json','Select detections JSON');
        if isequal(f,0), error('No JSON selected.'); end
        jsonFile = fullfile(p,f);
    end

    % Initialize ROS node
    rosinit('10.42.0.1'); % Pi ROS master
    import motion.*; import perception.*;

    %% -------------------- Tunables --------------------
    ho = 0.05;                    % hover height (m)
    tableZ = 0.000;               % <--- set your table top Z in *base* frame (m)
    minZ  = tableZ + 0.010;       % don't dip below table (+1 cm)
    pickZ_offset = 0.005;         % how much to push down from measured block top (safety)

    % Eye-to-hand extrinsics (camera pose in robot base frame) 4x4
    % >>>>>>> UPDATE THIS with your measured/calibrated T_base_cam <<<<<<
    T_base_cam = trotz(0) * trotx(-pi/2) * transl(0.10, 0.00, 0.65);

    %% -------------------- Read JSON detections --------------------
    J = jsondecode( fileread(jsonFile) );

    % Intrinsics (from file)
    fx = J.intrinsics.fx;  fy = J.intrinsics.fy;
    cx = J.intrinsics.cx;  cy = J.intrinsics.cy;

    detections = J.detections;    % array of structs
    if isempty(detections)
        rosshutdown; error('No detections in JSON.');
    end

    % Convert detections → base-frame block structs
    blocks = jsonDetectionsToBlocks(detections, fx, fy, cx, cy, T_base_cam, ...
                                    tableZ, minZ, pickZ_offset, ho);

    % (Optional) Sort blocks into canonical colour order if you want: RGB
    wantOrder = ["red","green","blue"];
    blocks = sortBlocksByColour(blocks, wantOrder);

    %% -------------------- Homing --------------------
    disp('=======================================');
    disp("Homing");
    [safetyStatePublisher, safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2; % INITIALISING
    send(safetyStatePublisher, safetyStateMsg);

    % Wait for OPERATING (4)
    pauseSub = rossubscriber('/dobot_magician/safety_status');
    fprintf('Waiting for OPERATING (4)...\n');
    while true
        pause(0.2);
        msg = pauseSub.LatestMessage;
        if ~isempty(msg) && msg.Data == 4
            fprintf('OPERATING.\n'); break
        end
    end
    disp('Homing complete.');
    disp('READY TO PICK AND PLACE');
    disp('=======================================');

    %% -------------------- Pick & Place loop --------------------
    for k = 1:numel(blocks)
        blk = blocks(k);

        disp('=======================================');
        fprintf('Picking and placing block %d:\n', k);
        fprintf('Colour: %s\n', blk.colour);
        fprintf('Pick position (base): [%.4f %.4f %.4f]\n', blk.blockPosition);
        fprintf('Hover position:       [%.4f %.4f %.4f]\n', blk.hoverPosition);
        fprintf('Orientation (rpy):    [%.2f %.2f %.2f]\n', blk.blockOrientation);

        move2goal(blk.hoverPosition, blk.blockOrientation);   % above block
        grabObject(blk.blockPosition, blk.blockOrientation);  % descend + close

        [goalPosition, goalOrientation] = goalForColours(blk.colour); % your existing logic
        move2goal(goalPosition, goalOrientation);              % go to bin/zone
        releaseObject(goalPosition);                           % open gripper

        move2goal([0.1; -0.1; 0.1], [0 0 0]); % safe pose
        fprintf('Block %d placed successfully.\n', k);
        disp('=======================================');
    end

    %% -------------------- Shutdown --------------------
    disp('Pick and place demo complete. Shutting down ROS...');
    rosshutdown;
    disp('=======================================');
end

%% ==================== Helpers ====================

function blocks = jsonDetectionsToBlocks(dets, fx, fy, cx, cy, T_base_cam, tableZ, minZ, pickZ_off, ho)
% Convert JSON detections to an array of block structs in *base* frame.
% Each det has fields:
%   .color (char), .centroid.u, .centroid.v, .depth_m, .corners (4x2)
%
% We back-project using pinhole model:
%   Xc = (u - cx)/fx * Z,  Yc = (v - cy)/fy * Z,  Zc = Z
% Then transform:  p_base = T_base_cam * [Xc; Yc; Zc; 1]

    blocks = struct('colour',{},'blockPosition',{},'hoverPosition',{},'blockOrientation',{});
    for i = 1:numel(dets)
        d = dets(i);
        if ~isfield(d,'depth_m') || isempty(d.depth_m)
            % If Z missing, skip (or set a default)
            warning('Detection %d (%s): no depth; skipping.', i, d.color); %#ok<*WNTAG>
            continue
        end
        u = d.centroid.u; v = d.centroid.v; Z = d.depth_m;

        % Camera frame point
        Xc = (u - cx)/fx * Z;
        Yc = (v - cy)/fy * Z;
        Zc = Z;

        % To base frame
        p_base_h = T_base_cam * [Xc; Yc; Zc; 1];
        p_base   = p_base_h(1:3);

        % Clamp/adjust Z to table/known surface with a small offset
        p_base(3) = max(p_base(3) - pickZ_off, minZ);

        % Orientation:
        %   simplest: flat pick (rpy=[0 0 0] in base). 
        %   If you want yaw from the quad, uncomment below to estimate yaw.
        rpy = [0 0 0];
        if isfield(d,'corners') && numel(d.corners)==4
            try
                rpy(3) = estimateYawFromCorners(d.corners); % radians
            catch
                % keep zero yaw on failure
            end
        end

        blk.colour         = string(d.color);
        blk.blockPosition  = p_base(:);
        blk.hoverPosition  = [p_base(1); p_base(2); p_base(3) + ho];
        blk.blockOrientation = rpy;
        blocks(end+1) = blk; %#ok<AGROW>
    end
end

function yaw = estimateYawFromCorners(corners)
% corners: 4x2 [u v] ordered TL,TR,BR,BL (as saved by Python)
% We infer the long edge direction in image coords and convert to a yaw.
% This is a heuristic: it assumes camera optical axis ~world -Z and small tilt.

    C = cell2mat(reshape(corners,1,[])); % handle if JSON decoded as cell
    if ~ismatrix(C)
        C = cell2mat(corners); % fallback
    end
    % Ensure 4x2
    if size(C,1) ~= 4
        C = reshape(C,4,2);
    end
    tl = C(1,:); tr = C(2,:);
    edge = tr - tl;              % image x-axis direction of top edge
    yaw_img = atan2(edge(2), edge(1));  % in image plane
    % Map image-plane angle to base yaw:
    % If camera is top-down and aligned, image +x ≈ base +x, +y ≈ base -y.
    % A minimal mapping is to negate to get CCW in base; tune if needed.
    yaw = -yaw_img;
end

function blocks = sortBlocksByColour(blocks, order)
    if isempty(blocks), return; end
    col = arrayfun(@(b) string(lower(b.colour)), blocks, 'UniformOutput', true);
    idx = zeros(size(col));
    for k = 1:numel(order)
        idx(col==order(k)) = k;
    end
    idx(idx==0) = numel(order)+1; % unknown to the end
    [~,p] = sort(idx);
    blocks = blocks(p);
end
