import perception.* 
import cali_img.*

function workspace = defineCheckerboardWorkspace(refImagePath, squareSize, originMode, originIndex)

    if nargin < 3 || isempty(originMode), originMode = 'corner'; end
    if nargin < 4, originIndex = []; end

    % 1) Intrinsics/distortion
    cam = getCameraCalibration();  
    intr = cam.intr;

    % 2) Read & undistort reference image
    I  = imread(refImagePath);
    Iu = undistortImage(I, intr);

    % 3) Detect checkerboard corners
    [imgPts, boardSize] = detectCheckerboardPoints(Iu);
    if isempty(imgPts)
        error('Checkerboard not detected. Check image, lighting, or board size.');
    end

    % 4) Generate default world points (board frame, Z=0)
    %    X across columns (boardSize(1)), Y across rows (boardSize(2))
    worldPts = generateCheckerboardPoints(boardSize, squareSize);

    % 5) Choose origin
    switch lower(originMode)
        case 'corner'
            if isempty(originIndex)
                % default: first corner in the returned ordering (bottom-left in image coordinates)
                % No shift needed; origin already at worldPts(1,:)
            else
                col = originIndex(1); row = originIndex(2);
                assert(col>=1 && col<=boardSize(1) && row>=1 && row<=boardSize(2), ...
                       'originIndex out of range');
                linearIdx = (row-1)*boardSize(1) + col;   % col=1..cols, row=1..rows
                originShift = worldPts(linearIdx,:);      % [X Y] at that internal corner
                worldPts(:,1:2) = worldPts(:,1:2) - originShift;
            end

        case 'center'
            % center the grid so that origin is the geometric center
            cols = boardSize(1); rows = boardSize(2);
            centerXY = [(cols-1)*squareSize/2, (rows-1)*squareSize/2];
            worldPts(:,1:2) = worldPts(:,1:2) - centerXY;

        otherwise
            error('originMode must be ''corner'' or ''center''');
    end

    % 6) Pose estimation (board wrt camera)
    %    Points are already undistorted because we used undistortImage.
    %    Use intrinsics-aware solver:
    [R, t] = estimateWorldCameraPose(imgPts, worldPts, intr, 'Confidence', 99.99, 'MaxReprojectionError', 1.5);
    % R,t here are *camera* wrt *worldPts*? In MATLAB, estimateWorldCameraPose returns
    % cameraPose relative to world. Convert to board->camera:
    R_cam_board = R';               % invert rotation
    t_cam_board = -R' * t(:);       % invert translation

    % 7) Homogeneous transforms
    T_cam_board = [R_cam_board, t_cam_board; 0 0 0 1];
    T_board_cam = inv(T_cam_board);

    workspace = struct( ...
        'intr', intr, ...
        'K', cam.K, ...
        'boardSize', boardSize, ...
        'squareSize', squareSize, ...
        'R', R_cam_board, ...
        't', t_cam_board(:).', ...
        'T_cam_board', T_cam_board, ...
        'T_board_cam', T_board_cam, ...
        'T_world_cam', T_board_cam, ...    % adopt board as workspace
        'T_cam_world', T_cam_board, ...
        'originMode', originMode, ...
        'originIndex', originIndex);

    figure; imshow(Iu); hold on;
    plot(imgPts(:,1), imgPts(:,2), 'g+'); title('Undistorted image + detected corners');
end
