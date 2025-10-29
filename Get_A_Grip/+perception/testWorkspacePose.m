import cali_img.*
import perception.*
import defineWorkspace.* 

function test_workspace_pose
clc; close all;

% --- Settings ---
baseName    = 'frame0000';   % no extension needed
squareSize  = 0.012;         % 12 mm in metres
originMode  = 'corner';
originIndex = [1,1];         % top-left internal corner

% --- 1) Check functions on path ---
req = {'defineCheckerboardWorkspace','getCameraCalibration'};
for i = 1:numel(req)
    if exist(req{i}, 'file') ~= 2
        error('Function "%s" not found on path. Add its folder: addpath(genpath(''%s''))', ...
              req{i}, pwd);
    end
end

% --- 2) Resolve image filename (try common extensions) ---
ex = {'.jpg','.jpeg','.png','.bmp','.tif','.tiff','.JPG','.PNG','.TIF'};
imgPath = '';
for k = 1:numel(ex)
    f = [baseName ex{k}];
    if exist(f,'file') == 2
        imgPath = f; break;
    end
end
if isempty(imgPath)
    % try wildcard search
    cand = dir([baseName '.*']);
    if ~isempty(cand)
        imgPath = fullfile(cand(1).folder, cand(1).name);
    else
        d = dir; names = {d.name};
        error('Could not find "%s" with common extensions. Files here:\n%s', ...
              baseName, strjoin(names, ', '));
    end
end
fprintf('Using image: %s\n', imgPath);

% --- 3) Try to read & show basic info ---
info = imfinfo(imgPath);
fprintf('Image: %dx%d, %s\n', info.Width, info.Height, info.Format);

% --- 4) Run workspace definition (with detailed error if it fails) ---
try
    ws = defineCheckerboardWorkspace(imgPath, squareSize, originMode, originIndex);
catch ME
    fprintf(2, '\nERROR during defineCheckerboardWorkspace:\n%s\n', ME.message);
    fprintf('Stack:\n'); disp(ME.stack);
    return;
end

% --- 5) Visual verification: plot undistorted image, corners, and XYZ triad ---
I  = imread(imgPath);
Iu = undistortImage(I, ws.intr);
figure('Name','Workspace verification'); imshow(Iu); hold on;

[imgPts, ~] = detectCheckerboardPoints(Iu);
if isempty(imgPts)
    warning('detectCheckerboardPoints failed on undistorted image (visual check only).');
else
    plot(imgPts(:,1), imgPts(:,2), 'g+', 'LineWidth', 1);
end

% Axis triad from origin
L = 0.06;  % 6 cm axis length
proj = @(Pw) worldToPixels(ws, Pw);
uvO = proj([0 0 0]);
uvX = proj([L 0 0]);
uvY = proj([0 L 0]);
uvZ = proj([0 0 L]);

plot([uvO(1) uvX(1)], [uvO(2) uvX(2)], 'r-', 'LineWidth', 2);
plot([uvO(1) uvY(1)], [uvO(2) uvY(2)], 'g-', 'LineWidth', 2);
plot([uvO(1) uvZ(1)], [uvO(2) uvZ(2)], 'b-', 'LineWidth', 2);
text(uvX(1), uvX(2), ' X', 'Color','r','FontWeight','bold');
text(uvY(1), uvY(2), ' Y', 'Color','g','FontWeight','bold');
text(uvZ(1), uvZ(2), ' Z', 'Color','b','FontWeight','bold');
title('Top-left origin: X=red, Y=green, Z=blue');

% --- 6) Save result for your pipeline ---
save('workspace_pose.mat','ws');
disp('Saved workspace_pose.mat (ws.T_world_cam, ws.T_cam_world, etc.).');

end

% ---- helper used above (same projection convention as before) ----
function uv = worldToPixels(ws, Pw)
    % Ensure Nx3 input
    Pw = reshape(Pw, [], 3);

    % Convert workspace -> camera
    n = size(Pw,1);
    Pw_h = [Pw, ones(n,1)];                 % homogeneous Nx4
    T = ws.T_cam_world';                    % explicit transpose first
    Pc_h = Pw_h * T;                        % apply transform
    Pc = Pc_h(:,1:3);                       % discard homogeneous component

    % Project to pixels
    uv = worldToImage(ws.intr, Pc);

    if size(uv,1) == 1
        uv = uv(1,:);                       % ensure row vector
    end
    ends