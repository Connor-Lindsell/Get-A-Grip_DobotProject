function camParam = getCameraCalibration()
%GETCAMERACALIBRATION  Return intrinsics/distortion from Bouguet calibration results.
% Copy fc, cc, alpha_c, kc, nx, ny directly from Calib_Results.m

    % ==== Paste from Calib_Results.m ====
    fc = [ 907.016155766797283 ; 912.211105886656696 ];
    cc = [ 660.156166957534424 ; 384.191277635435142 ];
    alpha_c = 0.000000000000000;
    kc = [ 0.167298999562626 ; -0.354312559662885 ; ...
           -0.003249389155066 ; 0.002937533008856 ; 0.000000000000000 ];
    nx = 1280;
    ny = 720;
    % ====================================

    % Assign clearer names for readability
    fx = fc(1);
    fy = fc(2);
    cx = cc(1);
    cy = cc(2);

    % Split distortion terms
    radial     = [kc(1) kc(2) kc(5)];  % [k1 k2 k3]
    tangential = [kc(3) kc(4)];        % [p1 p2]

    % Construct cameraIntrinsics object
    intr = cameraIntrinsics([fx fy], [cx cy], [ny nx], ...
        'Skew', alpha_c, ...
        'RadialDistortion', radial, ...
        'TangentialDistortion', tangential);

    % Build intrinsic matrix (3x3)
    K = intr.IntrinsicMatrix';

    % Output struct
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

