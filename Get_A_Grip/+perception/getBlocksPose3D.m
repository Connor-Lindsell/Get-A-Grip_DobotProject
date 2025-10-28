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
% =========================================================

function blocksPose = getBlocksPose3D()

    %% Properties 
    import perception.*

    camParam = getCameraCalibration;

    %%

    
    

    end
