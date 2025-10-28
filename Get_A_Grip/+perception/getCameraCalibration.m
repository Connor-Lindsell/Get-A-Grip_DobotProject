function camParam = getCameraCalibration()
%GETCAMERACALIBRATION  Return intrinsics/distortion from Bouguet-style results.
% Returns:
%   camParam.intr      : cameraIntrinsics object
%   camParam.K         : 3x3 intrinsic matrix (Computer Vision Toolbox convention)
%   camParam.kc        : [k1 k2 p1 p2 k3] Bouguet order
%   camParam.radial    : [k1 k2 k3]
%   camParam.tangential: [p1 p2]
%   camParam.imageSize : [ny nx]
%   camParam.focal     : [fx fy]
%   camParam.principal : [cx cy]
%   camParam.skew      : alpha_c

    % ==== Paste from Matlab Camera Calibration toolbox results ====
    fx = 692.609244167284487;
    fy = 690.065254803508310;

    cx = 284.905512041129725;
    cy = 244.626846325448014;

    alpha_c = 0.0; % skew
 
    % Bouguet kc = [k1 k2 p1 p2 k3]
    kc = [ 0.106697069970503 ; -0.250539104052457 ; ...
           0.000104056976962 ; -0.000248328870229 ; 0.0 ];

    nx = 1280; % width
    ny = 720;  % height
    % ========================================

    % Split distortion in MATLAB CVT order
    radial      = [kc(1) kc(2) kc(5)]; % [k1 k2 k3]
    tangential  = [kc(3) kc(4)];       % [p1 p2]

    % Construct cameraIntrinsics (preferred, lightweight and accepted by most CVT funcs)
    intr = cameraIntrinsics([fx fy], [cx cy], [ny nx], ...
        'Skew', alpha_c, ...
        'RadialDistortion', radial, ...
        'TangentialDistortion', tangential);

    % MATLAB stores IntrinsicMatrix as:
    % [fx  0   0
    %  0  fy   0
    %  cx cy   1]
    K = intr.IntrinsicMatrix';

    camParam = struct( ...
        'intr', intr, ...
        'K', K, ...
        'kc', kc(:).', ...
        'radial', radial, ...
        'tangential', tangential, ...
        'imageSize', [ny nx], ...
        'focal', [fx fy], ...
        'principal', [cx cy], ...
        'skew', alpha_c );

end
