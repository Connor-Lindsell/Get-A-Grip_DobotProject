import perception.*

% 1) Known camera intrinsics (use your getCameraCalibration)
camParam = getCameraCalibration(); 
intr = camParam.intr;

% 2) Define blockSize (edge length in meters)
blockSize = 0.04;   % 40 mm square

% 3) Define fake block corners (pixels) near image center
% Assume principal point around (640, 360)
cx = camParam.principal(1);
cy = camParam.principal(2);
half_px = 50;   % 100-pixel wide square

block(1).corners = [cx - half_px, cy - half_px;   % TL
                    cx + half_px, cy - half_px;   % TR
                    cx + half_px, cy + half_px;   % BR
                    cx - half_px, cy + half_px];  % BL
block(1).meta.id = 1;

% 4) Hand-eye transform: camera→robot
%    Rotate 90° about Z, translate +0.2 m in X
Rz = axang2rotm([0 0 1 deg2rad(90)]);
T_robot_from_cam = [Rz [0.2; 0; 0]; 0 0 0 1];

% 5) Call your function
poses = getBlocksPose3D(T_robot_from_cam, blockSize, block);

% 6) Print results
disp('--- Camera->Block pose ---');
disp(poses(1).T_cam_from_block);

disp('--- Robot->Block pose ---');
disp(poses(1).T_robot_from_block);

disp('Robot-frame position [m]:');
disp(poses(1).position_robot);

disp('Robot-frame quaternion [w x y z]:');
disp(poses(1).quaternion_robot);
