%% DobotKinematics.m
% Simulation model and utility functions for a Dobot-style 4-DOF arm.
% The class encapsulates the Peter Corke Robotics Toolbox SerialLink model
% and provides helper methods for forward/inverse kinematics, motion
% planning, and safety checks.  This logic can be reused later for the
% physical robot implementation.

classdef DobotKinematics
    %% --------------------------------------------------------------------
    %  Properties
    %  --------------------------------------------------------------------
    properties
        robot          % SerialLink model
        toolT          % SE3 tool transform
        qHome          % nominal home posture
        qlim           % joint limits [min max] per row
        ws             % workspace box [xmin xmax; ymin ymax; zmin zmax]
        tableZ = 0.0   % table plane height
        safetyClear = 0.01 % min clearance above table [m]
        ikMask = [1 1 1 0 0 1] % solve x y z yaw only (keep pitch, roll fixed)
    end

    methods
        %% ----------------------------------------------------------------
        %  Constructor and model setup
        %  ----------------------------------------------------------------
        function self = DobotKinematics()
            % Build a simple 4 DOF RRR+roll Dobot style arm
            % DH: [theta d a alpha] using standard DH
            % Replace these with your measured or provided parameters
            L_base = 0.080;  % base offset to shoulder axis
            L1 = 0.135;
            L2 = 0.160;
            Lw = 0.040;      % small wrist link before tool

            % Joint limits (approx). Update per your spec.
            q1lim = deg2rad([-150 150]);
            q2lim = deg2rad([-10 85]);
            q3lim = deg2rad([-100 100]);
            q4lim = deg2rad([-180 180]);
            self.qlim = [q1lim; q2lim; q3lim; q4lim];

            % Workspace box. Tweak as needed.
            self.ws = [-0.25 0.25; -0.25 0.25; 0.00 0.30];

            % Define links
            % For a simple vertical shoulder, we use an offset up L_base on link 1
            L(1) = Link('revolute', 'd', L_base, 'a', 0.0,  'alpha', pi/2, 'qlim', q1lim);  % base yaw
            L(2) = Link('revolute', 'd', 0.0,    'a', L1,   'alpha', 0.0,  'qlim', q2lim);  % shoulder
            L(3) = Link('revolute', 'd', 0.0,    'a', L2,   'alpha', 0.0,  'qlim', q3lim);  % elbow
            L(4) = Link('revolute', 'd', 0.0,    'a', Lw,   'alpha', 0.0,  'qlim', q4lim);  % wrist roll

            self.robot = SerialLink(L, 'name', 'DobotSim');

            % Tool frame: fingertip below wrist. Add your gripper length here.
            L_tool = 0.060; % distance wrist center to fingertip along negative Z
            self.toolT = transl(0,0,-L_tool);
            self.robot.tool = self.toolT;

            % Home posture
            self.qHome = [0, deg2rad(20), deg2rad(30), 0];
        end

        %% ----------------------------------------------------------------
        %  Visualisation helpers
        %  ----------------------------------------------------------------
        function showModel(self)
            figure; hold on; axis equal;
            self.robot.plot(self.qHome, 'workspace', self.ws(:).', 'trail', 'r-');
            xlabel X; ylabel Y; zlabel Z; grid on;
            title('Dobot model')
            zlim([self.ws(3,1) self.ws(3,2)]);
        end

        %% ----------------------------------------------------------------
        %  Core kinematics methods
        %  ----------------------------------------------------------------
        function T = fk(self, q)
            T = self.robot.fkine(q);
        end

        function [q, ok, info] = ik(self, Ttarget, q0, elbowSign)
            % IK solving x y z yaw with mask, keep pitch and roll fixed
            % elbowSign: +1 for elbow up, -1 for elbow down (hint to initial guess)
            if nargin < 3 || isempty(q0)
                q0 = self.qHome;
            end
            if nargin < 4, elbowSign = +1; end

            % Provide a better q0 for the elbow using geometric hint
            q0(3) = elbowSign * abs(q0(3));

            try
                q = self.robot.ikine(Ttarget, q0, self.ikMask, 'tol', 1e-6, 'ilimit', 100, 'rlimit', 50);
                ok = self.isWithinLimits(q) && self.isWithinWorkspace(Ttarget);
                info = "ok";
                if ~ok
                    info = "limit or workspace violation";
                end
            catch ME
                q = nan(1, self.robot.n);
                ok = false;
                info = ME.message;
            end
        end

        %% ----------------------------------------------------------------
        %  Safety checks and workspace helpers
        %  ----------------------------------------------------------------
        function tf = isWithinLimits(self, q)
            tf = all(q(:) >= self.qlim(:,1) - 1e-9) && all(q(:) <= self.qlim(:,2) + 1e-9);
        end

        function tf = isWithinWorkspace(self, T)
            p = transl(T);
            tf = p(1) >= self.ws(1,1) && p(1) <= self.ws(1,2) && ...
                 p(2) >= self.ws(2,1) && p(2) <= self.ws(2,2) && ...
                 p(3) >= self.ws(3,1) && p(3) <= self.ws(3,2);
            % table clearance
            tf = tf && (p(3) >= self.tableZ + self.safetyClear);
        end

        %% ----------------------------------------------------------------
        %  Motion planning utilities
        %  ----------------------------------------------------------------
        function qs = planJtraj(self, qStart, qGoal, steps)
            if nargin < 4, steps = 50; end
            qs = jtraj(qStart, qGoal, steps);
        end

        function Ts = planLinearCartesian(self, Tstart, Tend, steps)
            if nargin < 4, steps = 25; end
            Ts = ctraj(Tstart, Tend, steps);
        end

        function [ok, report] = safetyCheckPath(self, qs)
            % Sample FK along qs, check workspace and table clearance
            ok = true; report = "";
            for k = 1:size(qs,1)
                T = self.fk(qs(k,:));
                if ~self.isWithinWorkspace(T)
                    ok = false;
                    report = sprintf('Violation at step %d', k);
                    return
                end
            end
        end

        %% ----------------------------------------------------------------
        %  Pose utility
        %  ----------------------------------------------------------------
        function T = poseFromXYZYaw(self, xyz, yaw)
            % Tool vertical, yaw about base Z
            T = transl(xyz) * trotz(yaw);
        end
    end
end
