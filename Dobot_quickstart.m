%% Dobot Magician — MATLAB Quickstart + Examples
% This script connects to the Dobot's ROS master, checks safety state,
% performs a safe initialise/home, then shows example commands:
%  - Read joint states and end-effector pose
%  - Move by joints or by pose
%  - Use suction/gripper
%  - Read/set IO ports
%  - Use linear rail and conveyor
%
% Startup summary:
% 1) Power RPi + Dobot, wait for hotspot "ubiquityrobotXXXX". Connect to it.
% 2) On the RPi: driver is launched with roslaunch dobot_magician_driver dobot_magician.launch
% 3) From MATLAB: connect with rosinit('10.42.0.1'), verify node exists,
%    home the robot by setting safety state to INITIALISING, then wait for OPERATING.
%
% NOTE: Replace IP if your setup differs.

clc; clear; close all;

%% --- CONFIG ---
rosIP = '10.42.0.1';   % Default ROS master on the Dobot RPi
homeOnStart = true;    % Set true to auto-home at startup
waitTimeout = 60;      % Seconds to wait for status transitions

%% --- 0) Connect to ROS master on the Dobot RPi ---
% If a ROS session already exists, shut it down to avoid conflicts.
if robotics.ros.internal.Global.isNodeActive
    rosshutdown;
end

fprintf('Connecting to ROS master at %s ...\n', rosIP);
rosinit(rosIP); % Connect to the Dobot ROS master

% Quick sanity check: list nodes (optional visual check in Command Window)
disp('ROS nodes on network:'); rosnode('list');

%% --- 1) Safety status interface ---
% Create publisher to command target safety status, and a subscriber to read current status.
[safetyPub, safetyMsg] = rospublisher('/dobot_magician/target_safety_status', 'std_msgs/Int32');
safetySub = rossubscriber('/dobot_magician/safety_status', 'std_msgs/Int32');

% Helper to fetch current safety code
getSafety = @() ( receive(safetySub, 2).Data ); % returns an integer
% Codes of interest: INITIALISING=2, ESTOPPED=3, OPERATING=4, STOPPED=6

cur = tryGetSafety(getSafety);
fprintf('Current safety status code = %d\n', cur);

% If requested, initialise/home the robot at startup.
if homeOnStart
    if cur ~= 4 % not OPERATING
        fprintf('Initialising (homing) robot...\n');
        safetyMsg.Data = 2; % INITIALISING
        send(safetyPub, safetyMsg);

        ok = waitForSafetyStatus(getSafety, 4, waitTimeout); % wait for OPERATING
        if ~ok
            warning('Timed out waiting for OPERATING. Check driver and E-stop.');
        else
            fprintf('Robot homed. Status is OPERATING.\n');
        end
    else
        fprintf('Robot already in OPERATING.\n');
    end
end

%% --- 2) Read robot state (joints and pose) ---
% Joint states
jointSub = rossubscriber('/dobot_magician/joint_states', 'sensor_msgs/JointState');
jmsg = receive(jointSub, 2);
currentJoints = jmsg.Position(:).'; % [q1 q2 q3 q4]
fprintf('Current joints: [%.3f %.3f %.3f %.3f]\n', currentJoints);

% End-effector pose
eeSub = rossubscriber('/dobot_magician/end_effector_poses', 'geometry_msgs/PoseStamped');
eemsg = receive(eeSub, 2);
p = [eemsg.Pose.Position.X, eemsg.Pose.Position.Y, eemsg.Pose.Position.Z];
q = [eemsg.Pose.Orientation.W, eemsg.Pose.Orientation.X, ...
     eemsg.Pose.Orientation.Y, eemsg.Pose.Orientation.Z];
[r,pit,yw] = quat2eul(q); % radians
fprintf('EE position [m]: [%.3f %.3f %.3f], RPY [rad]: [%.3f %.3f %.3f]\n', p, r, pit, yw);

%% --- 3) Move by joints (single waypoint) ---
[targetJointPub, targetJointMsg] = rospublisher('/dobot_magician/target_joint_states', 'trajectory_msgs/JointTrajectory');
pt = rosmessage('trajectory_msgs/JointTrajectoryPoint');

% Example target joints: change as needed, keep within Dobot limits
targetJoints = [0, 0.40, 0.30, 0];  % radians
pt.Positions = targetJoints;
targetJointMsg.Points = pt;

% Send the command
fprintf('Commanding joint move...\n');
send(targetJointPub, targetJointMsg);
pause(2); % allow motion

%% --- 4) Move by end-effector pose (position + orientation) ---
[targetEEPub, targetEEMsg] = rospublisher('/dobot_magician/target_end_effector_pose', 'geometry_msgs/Pose');

% Example pose in metres and radians at default tool flange
eePos  = [0.20, 0.00, 0.10];   % x y z
eeRPY  = [0.00, 0.00, 0.00];   % roll pitch yaw
eeQuat = eul2quat(eeRPY);      % [w x y z]

targetEEMsg.Position.X = eePos(1);
targetEEMsg.Position.Y = eePos(2);
targetEEMsg.Position.Z = eePos(3);
targetEEMsg.Orientation.W = eeQuat(1);
targetEEMsg.Orientation.X = eeQuat(2);
targetEEMsg.Orientation.Y = eeQuat(3);
targetEEMsg.Orientation.Z = eeQuat(4);

fprintf('Commanding EE pose move...\n');
send(targetEEPub, targetEEMsg);
pause(2);

%% --- 5) Tool control: suction + gripper ---
% Read current tool state
toolSub = rossubscriber('/dobot_magician/tool_state', 'std_msgs/Int32MultiArray');
tmsg = receive(toolSub, 2);
fprintf('Current tool raw state: '); disp(tmsg.Data.');

% Set tool state
[toolPub, toolMsg] = rospublisher('/dobot_magician/target_tool_state', 'std_msgs/Int32MultiArray');

% Examples:
% Suction only: [1] to turn ON, [0] to turn OFF (driver accepts 1-el form)
toolMsg.Data = int32(1);  % vacuum ON
send(toolPub, toolMsg); pause(1);

toolMsg.Data = int32(0);  % vacuum OFF
send(toolPub, toolMsg); pause(1);

% Pump + gripper: [pump, gripper] where 0=open 1=close
toolMsg.Data = int32([1, 0]); % pump ON, gripper OPEN
send(toolPub, toolMsg); pause(1);

toolMsg.Data = int32([1, 1]); % pump ON, gripper CLOSE
send(toolPub, toolMsg); pause(1);

toolMsg.Data = int32([0, 0]); % pump OFF, gripper OPEN
send(toolPub, toolMsg); pause(1);

%% --- 6) IO ports: read + set example ---
ioSub = rossubscriber('/dobot_magician/io_data', 'std_msgs/Int32MultiArray');
raw = receive(ioSub, 2).Data;   % driver specific layout
ioModes  = raw(2:21);           % modes for ports 1..20
ioValues = raw(23:42);          % values for ports 1..20

port = 1; % example port index
fprintf('Port %d mode=%d value=%d\n', port, ioModes(port), ioValues(port));

% Set IO: Data = [address, ioMux, data]  where ioMux=1 for DIGITAL OUTPUT
[ioPub, ioMsg] = rospublisher('/dobot_magician/target_io_state', 'std_msgs/Int32MultiArray');
ioMsg.Data = int32([port, 1, 1]); % set HIGH
send(ioPub, ioMsg); pause(0.5);
ioMsg.Data = int32([port, 1, 0]); % set LOW
send(ioPub, ioMsg);

%% --- 7) Linear rail: enable + move ---
[railEnPub, railEnMsg] = rospublisher('/dobot_magician/target_rail_status', 'std_msgs/Bool');
railEnMsg.Data = true;  % enable rail
send(railEnPub, railEnMsg);
fprintf('Rail enabled.\n');

% When enabling for the first time after power up, home again to zero rail
if homeOnStart
    fprintf('Re-initialising to include rail homing...\n');
    safetyMsg.Data = 2; % INITIALISING
    send(safetyPub, safetyMsg);
    waitForSafetyStatus(getSafety, 4, waitTimeout);
end

% Read rail position
railSub = rossubscriber('/dobot_magician/rail_position', 'std_msgs/Float32');
rpos = receive(railSub, 2).Data;
fprintf('Current rail position: %.3f\n', rpos);

% Command rail position (driver units, commonly metres)
[railPosPub, railPosMsg] = rospublisher('/dobot_magician/target_rail_position', 'std_msgs/Float32');
railPosMsg.Data = 0.50; % move to 0.5
send(railPosPub, railPosMsg); pause(2);

%% --- 8) Conveyor belt: start/stop ---
[beltPub, beltMsg] = rospublisher('/dobot_magician/target_e_motor_state', 'std_msgs/Int32MultiArray');

enable = 1; ticksPerSec = 15000; % recommended upper bound unless characterised
beltMsg.Data = int32([enable, ticksPerSec]);
fprintf('Starting conveyor at %d ticks/s...\n', ticksPerSec);
send(beltPub, beltMsg); pause(2);

enable = 0; ticksPerSec = 0;
beltMsg.Data = int32([enable, ticksPerSec]);
fprintf('Stopping conveyor.\n');
send(beltPub, beltMsg);

%% --- 9) Software E-stop example (use with care) ---
% To immediately halt motion and IO, publish ESTOPPED then re-initialise to recover.
% safetyMsg.Data = 3; % ESTOPPED
% send(safetyPub, safetyMsg);

%% --- Helpers (local functions) ---
function ok = waitForSafetyStatus(getSafetyFcn, target, timeoutSec)
    t0 = tic; ok = false;
    while toc(t0) < timeoutSec
        s = tryGetSafety(getSafetyFcn);
        if ~isempty(s) && s == target
            ok = true; return;
        end
        pause(0.5);
    end
end

function s = tryGetSafety(getSafetyFcn)
    try
        s = getSafetyFcn();
    catch
        s = [];
    end
end
