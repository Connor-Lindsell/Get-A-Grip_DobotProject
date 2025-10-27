% ========================================================================
% IBVS.m
% 
% Implements Image Based Visual Servoing (IBVS) to move the robot's end
% effector to a desired position based on visual feedback from a camera.
%
% Inputs:
%   enfEffectorRefFeatures - A Nx2 matrix of current image feature of the reference
%                           position of the end effector.
%   desiredRefFeatures - A Nx2 matrix of desired image features for the end effector.
%   cameraParams - A structure containing the camera's intrinsic parameters.
% =========================================================================

function IBVS(endEffectorRefFeatures, desiredRefFeatures, cameraParams)


% NOT FINISHED: Example usage with user-defined features below
% To use with actual robot/camera, replace the feature input sections with
% code that extracts features from the camera feed.
% TO DO: integrate with robot control commands to move the end effector.
% TO DO: make reshape use move to goal position might need a helper function this code animates atm 

    %% Parameters (edit as needed)
    f      = cameraParams.f;     % focal length [px]
    Z      = cameraParams.Z;      % (assumed known) depth for all points
    lambda = 0.1;     % control gain
    numSteps = 100;   % animation iterations
    dt     = 0.5;     % integration step in feature space

    % Principal point (usually image center if you had an image; choose any reference)
    u0 = CameraParams.U0;  % e.g., 
    v0 = CameraParams.V0;  % e.g.,

    %% === USER INPUT: observed & desired feature pixel coordinates (Nx2) ===
    % Example: a square (replace with your own points)
    feat_uv = [380 260;   % observed TL
            520 250;   % observed TR
            530 390;   % observed BR
            370 400];  % observed BL          % <-- EDIT these

    % Option A: directly specify desired pixels (Nx2)
    % des_uv  = [u0-65 v0-65;
    %            u0+65 v0-65;
    %            u0+65 v0+65;
    %            u0-65 v0+65];                      % <-- EDIT these

    % Option B (alternative): build a centered square of side S_des
    S_des = 130; half = S_des/2;
    des_uv = [u0-half, v0-half;
            u0+half, v0-half;
            u0+half, v0+half;
            u0-half, v0+half];

    % Sanity checks
    assert(size(feat_uv,2)==2 && size(des_uv,2)==2, 'feat_uv and des_uv must be Nx2');
    assert(size(feat_uv,1)==size(des_uv,1), 'feat_uv and des_uv must have same N');
    N = size(feat_uv,1);
    assert(N>=3, 'Need at least 3 points for a well-conditioned L');

    %% Initial diagnostics (before animation)
    xy_obs = [(feat_uv(:,1)-u0)/f, (feat_uv(:,2)-v0)/f];  % normalize by intrinsics
    xy_des = [(des_uv(:,1)-u0)/f,  (des_uv(:,2)-v0)/f];
    e = reshape((xy_obs - xy_des).', [], 1);              % 2N×1 stacked error
    L = [];
    for i = 1:N, L = [L; Lx(xy_obs(i,1), xy_obs(i,2), Z)]; end %#ok<AGROW>
    v = -lambda * pinv(L) * e;
    fprintf('\n--- Initial Visual Servoing Diagnostics ---\n');
    disp('Detected corners (u,v):'); disp(feat_uv);
    disp('Desired corners  (u,v):'); disp(des_uv);
    disp('Feature error (obs - des) in normalized coords:'); disp(e');
    fprintf('Linear  [vx vy vz] = %.4f  %.4f  %.4f\n', v(1),v(2),v(3));
    fprintf('Angular [wx wy wz] = %.4f  %.4f  %.4f\n', v(4),v(5),v(6));
    disp('v:'); disp(v);

    %% Plot setup (no image background)
    figure('Name','IBVS (User-Input Features)','NumberTitle','off'); hold on; grid on; axis equal;
    % Choose axes limits around both sets of points
    all_u = [feat_uv(:,1); des_uv(:,1)]; all_v = [feat_uv(:,2); des_uv(:,2)];
    pad = 40;
    xlim([min(all_u)-pad, max(all_u)+pad]); ylim([min(all_v)-pad, max(all_v)+pad]); set(gca,'YDir','reverse');
    % Polylines (wrap to first point for visualization)
    wrap = @(M)[M; M(1,:)];
    hObs = plot(wrap(feat_uv(:,1)), wrap(feat_uv(:,2)), 'bo-','LineWidth',2);
    hDes = plot(wrap(des_uv(:,1)),  wrap(des_uv(:,2)),  'r--','LineWidth',2);
    legend('Observed','Desired','Location','southoutside');
    title('IBVS: Observed features converging to Desired');

    %% Iterative IBVS update (feature-space integration)
    for k = 1:numSteps
        % Normalize, error, interaction matrix
        xy_obs = [(feat_uv(:,1)-u0)/f, (feat_uv(:,2)-v0)/f];
        xy_des = [(des_uv(:,1)-u0)/f,  (des_uv(:,2)-v0)/f];
        e = reshape((xy_obs - xy_des).', [], 1);      % 2N×1

        L = [];
        for i = 1:N, L = [L; Lx(xy_obs(i,1), xy_obs(i,2), Z)]; end %#ok<AGROW>

        % Camera twist (classical IBVS law)
        v = -lambda * pinv(L) * e;

        % Optional concise logging
        if mod(k,5)==0 || k==1 || k==numSteps
            fprintf('Step %3d | ||e||=%.4f | vz=%.3f | wz=%.3f\n', k, norm(e), v(3), v(6));
        end

        % Feature evolution in image space: s_{k+1} = s_k + (L v) dt
        sdot  = reshape(L * v, 2, []).';  % N×2 from (2N×1)
        xy_obs = xy_obs + dt * sdot;

        % Back to pixels
        feat_uv = [xy_obs(:,1)*f + u0, xy_obs(:,2)*f + v0];

        % Update plot
        set(hObs,'XData',wrap(feat_uv(:,1)),'YData',wrap(feat_uv(:,2)));
        drawnow; pause(0.03);
    end
        
end
