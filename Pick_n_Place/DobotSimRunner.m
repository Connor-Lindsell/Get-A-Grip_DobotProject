%% DobotSimRunner.m
% High-level simulation harness for testing Dobot motion logic.  The class
% wraps planning of approach/contact/retreat waypoints, runs joint or
% Cartesian interpolation, performs safety checks, and animates the SerialLink
% model.  The intent is to mirror the structure that will eventually drive
% the physical robot controller.

classdef DobotSimRunner
    %% --------------------------------------------------------------------
    %  Properties
    %  --------------------------------------------------------------------
    properties
        kin            % DobotKinematics instance
        qCurrent       % current configuration during the simulation
        fig            % figure handle for the visualisation
        approachDZ = 0.05
        contactDZ  = 0.04
        moveStepsJoint = 60
        moveStepsCart  = 25
        elbowSign = +1  % +1 elbow up, -1 elbow down
    end

    methods
        %% ----------------------------------------------------------------
        %  Construction and scene setup
        %  ----------------------------------------------------------------
        function self = DobotSimRunner()
            self.kin = DobotKinematics();
            self.qCurrent = self.kin.qHome;
        end

        function build(self)
            self.fig = figure('Name', 'Dobot pick and place sim');
            hold on; axis equal; grid on;
            self.kin.robot.plot(self.qCurrent, 'workspace', self.kin.ws(:).', 'trail', 'r-');
            xlabel X; ylabel Y; zlabel Z;
            zlim([self.kin.ws(3,1) self.kin.ws(3,2)]);
            title('Dobot pick and place simulation')
            % draw table plane
            self.drawTable();
        end

        function drawTable(self)
            % Draw a faint table plane
            x = linspace(self.kin.ws(1,1), self.kin.ws(1,2), 2);
            y = linspace(self.kin.ws(2,1), self.kin.ws(2,2), 2);
            [X,Y] = meshgrid(x,y);
            Z = self.kin.tableZ * ones(size(X));
            surf(X,Y,Z, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', [0.6 0.6 0.6]);
        end

        %% ----------------------------------------------------------------
        %  Waypoint generation
        %  ----------------------------------------------------------------
        function seq = setWaypoints(self, A_xyz, A_yaw, B_xyz, B_yaw)
            % Build approach, contact, retreat transforms for A and B
            A_app = A_xyz; A_app(3) = A_xyz(3) + self.approachDZ;
            A_con = A_xyz; A_con(3) = A_xyz(3) + max(self.kin.safetyClear, 0) - self.contactDZ;
            A_ret = A_app;

            B_app = B_xyz; B_app(3) = B_xyz(3) + self.approachDZ;
            B_con = B_xyz; B_con(3) = B_xyz(3) + max(self.kin.safetyClear, 0) - self.contactDZ;
            B_ret = B_app;

            T = @(p,y) self.kin.poseFromXYZYaw(p, y);
            seq = { ...
                T(A_app, A_yaw);
                T(A_con, A_yaw);
                T(A_ret, A_yaw);
                T(B_app, B_yaw);
                T(B_con, B_yaw);
                T(B_ret, B_yaw) ...
            };
        end

        %% ----------------------------------------------------------------
        %  Simulation execution
        %  ----------------------------------------------------------------
        function out = runPickPlace(self, seq, useCartesianSegments)
            % Solve IK for each waypoint, plan motion, safety check, animate
            if nargin < 3, useCartesianSegments = false; end
            nW = numel(seq);
            qTargets = zeros(nW, self.kin.robot.n);
            okAll = true; why = strings(nW,1);

            % IK for each pose
            for i = 1:nW
                [q, ok, info] = self.kin.ik(seq{i}, self.qCurrent, self.elbowSign);
                qTargets(i,:) = q;
                okAll = okAll && ok;
                why(i) = string(info);
            end

            out.qTargets = qTargets;
            out.ikOk = okAll;
            out.ikInfo = why;

            if ~okAll
                warning('Some waypoints failed IK or safety. Details:'); disp(why);
                return
            end

            % Plan and animate segment by segment
            for i = 1:nW
                qGoal = qTargets(i,:);
                if useCartesianSegments && i > 1
                    % Cartesian interpolation between last and next task frames
                    Tstart = self.kin.fk(self.qCurrent);
                    Tend   = seq{i};
                    Ts = self.kin.planLinearCartesian(Tstart, Tend, self.moveStepsCart);
                    % Map each T to IK then animate
                    qs = zeros(self.moveStepsCart, self.kin.robot.n);
                    for k = 1:self.moveStepsCart
                        [qk, ok] = self.kin.ik(Ts(:,:,k), self.qCurrent, self.elbowSign);
                        if ~ok
                            warning('Cartesian segment IK failed at step %d. Falling back to joint space.', k);
                            qs = self.kin.planJtraj(self.qCurrent, qGoal, self.moveStepsJoint);
                            break
                        end
                        qs(k,:) = qk;
                        self.qCurrent = qk;
                    end
                else
                    % Joint trajectory
                    qs = self.kin.planJtraj(self.qCurrent, qGoal, self.moveStepsJoint);
                end

                % Safety check along path
                [safe, report] = self.kin.safetyCheckPath(qs);
                if ~safe
                    warning('Path safety violation: %s', report);
                    out.safe = false; out.violReport = report;
                    return
                end

                % Animate
                for k = 1:size(qs,1)
                    self.kin.robot.animate(qs(k,:));
                    drawnow;
                end
                self.qCurrent = qGoal;
            end

            out.safe = true;
            out.violReport = "";
        end
    end
end
