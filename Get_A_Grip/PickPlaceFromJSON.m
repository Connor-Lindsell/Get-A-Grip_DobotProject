function PickPlaceFromJSON(detectionsJson, handeyeJson)
% PickPlaceFromJSON
% Drive Dobot pick & place using RealSense detections + hand-eye calibration.
%
% Usage:
%   PickPlaceFromJSON('detections_...json', 'config/handeye.json')
%   % If args omitted, file pickers will open.
%
% Requires your existing motion helpers:
%   move2goal(position3x1, rpy1x3)
%   grabObject(position3x1, rpy1x3)
%   releaseObject(position3x1)
%   goalForColours(colourString)

    % -------------------- Files --------------------
    if nargin < 1 || ~isfile(detectionsJson)
        [f,p] = uigetfile('*.json','Select detections JSON'); 
        assert(~isequal(f,0),'No detections JSON selected.');
        detectionsJson = fullfile(p,f);
    end
    if nargin < 2 || ~isfile(handeyeJson)
        [f,p] = uigetfile('*.json','Select handeye.json');
        assert(~isequal(f,0),'No handeye.json selected.');
        handeyeJson = fullfile(p,f);
    end

    % -------------------- Load inputs --------------------
    D   = jsondecode(fileread(detectionsJson));
    fx  = D.intrinsics.fx;  fy = D.intrinsics.fy;
    cx  = D.intrinsics.cx;  cy = D.intrinsics.cy;

    T_rc = load_Trc_from_json(handeyeJson);      % 4x4 (camera -> base)

    % -------------------- Tunables --------------------
    ho    = 0.050;        % hover height above pick (m)
    tableZ = -0.080;      % table Z in base frame (adjust to your setup)
    minZ   = tableZ + 0.010;  % don't go below table + 10 mm
    push   = 0.005;       % push-down from measured top when grasping (m)

    % -------------------- Build blocks from detections --------------------
    dets = D.detections;
    assert(~isempty(dets),'No detections in JSON.');
    blocks = [];
    for i = 1:numel(dets)
        di = dets(i);
        if ~isfield(di,'depth_m') || isempty(di.depth_m) || di.depth_m <= 0
            continue
        end
        % Back-project to camera frame, then transform to base
        u = di.centroid.u; v = di.centroid.v; Z = di.depth_m;
        p_cam  = backproject_uvZ_to_cam(u, v, Z, fx, fy, cx, cy);   % [Xc;Yc;Zc;1]
        p_base = T_rc * p_cam;                                      % camera -> base

        % Z handling
        p_pick = p_base(1:3);
        p_pick(3) = max(p_pick(3) - push, minZ);

        % Orientation (flat), optional yaw from corners
        rpy = [0 0 0];
        if isfield(di,'corners') && numel(di.corners) == 4
            try
                rpy(3) = estimateYawFromCorners(di.corners);  % radians
            catch
                % ignore if any issue
            end
        end

        blk.colour          = string(lower(di.color));
        blk.blockPosition   = p_pick(:);
        blk.hoverPosition   = [p_pick(1); p_pick(2); p_pick(3)+ho];
        blk.blockOrientation= rpy;
        blocks = [blocks, blk]; %#ok<AGROW>
    end
    assert(~isempty(blocks),'No usable detections with valid depth.');

    % Optional: order RGB
    order = ["red","green","blue"];
    cols  = arrayfun(@(b) string(lower(b.colour)), blocks);
    [~,perm] = sort(arrayfun(@(c) find([order,"zzz"]==c,1,'first'), cols));
    blocks = blocks(perm);

    % -------------------- ROS: home & wait for OPERATING(4) --------------------
    rosinit('10.42.0.1');
    import motion.*
    import perception.*
    cleanupObj = onCleanup(@() safe_ros_shutdown());

    [safetyStatePublisher, safetyStateMsg ] = rospublisher ('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2; % INITIALISING
    send ( safetyStatePublisher , safetyStateMsg );

    msgSafety = rossubscriber('/dobot_magician/safety_status');

    fprintf('[PickPlace] Waiting for OPERATING (4)...\n');
    t0 = tic;
    while toc(t0) < 15
        pause(0.2);
        m = msgSafety.LatestMessage;
        if ~isempty(m) && m.Data == 4
            fprintf('[PickPlace] OPERATING.\n');
            break
        end
    end

    % -------------------- Execute pick & place --------------------
    for k = 1:numel(blocks)
        b = blocks(k);
        fprintf('\n[PickPlace] Block %d: %s\n', k, b.colour);
        fprintf('  pick  = [%.3f %.3f %.3f]\n', b.blockPosition);
        fprintf('  hover = [%.3f %.3f %.3f]\n', b.hoverPosition);

        move2goal(b.hoverPosition, b.blockOrientation);
        grabObject(b.blockPosition, b.blockOrientation);

        [goalP, goalRpy] = goalForColours(b.colour);   % your existing mapping
        move2goal(goalP, goalRpy);
        releaseObject(goalP);

        move2goal([0.10; -0.10; 0.10], [0 0 0]);      % safe pose
    end

    fprintf('\n[PickPlace] Done.\n');
end

% ======================== Helpers ========================

function safe_ros_shutdown()
    try, rosshutdown; catch, end
end

function T = load_Trc_from_json(handeyeFile)
    J = jsondecode(fileread(handeyeFile));
    T = J.T_rc;
    % If T comes in as nested cells, normalize to 4x4 double:
    if iscell(T)
        T = reshape([T{:}],4,4)'; 
    end
    validateattributes(T, {'double','single'}, {'size',[4,4]});
end

function p_cam = backproject_uvZ_to_cam(u, v, Z, fx, fy, cx, cy)
    X = (u - cx) / fx * Z;
    Y = (v - cy) / fy * Z;
    p_cam = [X; Y; Z; 1];
end

function yaw = estimateYawFromCorners(corners)
    % corners: 4x2 [u v] in TL,TR,BR,BL order (as saved by your Python)
    C = corners;
    if iscell(C), C = cell2mat(C); end
    if size(C,1) ~= 4, C = reshape(C,4,2); end
    tl = C(1,:); tr = C(2,:);
    e  = tr - tl;
    yaw_img = atan2(e(2), e(1));
    % Simple mapping from image plane to base yaw (tune if your cam is tilted):
    yaw = -yaw_img;
end
