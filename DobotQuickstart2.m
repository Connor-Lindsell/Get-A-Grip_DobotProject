%% DoBot Start


%% 3.1 

rosnode list
% Expect to see: ’/ dobot_magician / dobot_magician_node ’

%% ================== 3.2 Initialise connection ==================
% To establish a ROS session from MATLAB, invoke rosinit with the ROS master’s IP address.
% If the driver runs on the same machine as MATLAB, call rosinit without arguments.
rosinit ('10.42.0.1');

rosnode list
% Expect to see: ’/ dobot_magician / dobot_magician_node ’

%% ================== 3.3 Saftey Status ==================

% Status Code
% INVALID       0
% DISCONNECTED  1
% INITIALISING  2
% ESTOPPED      3
% OPERATING     4
% PAUSED        5
% STOPPED       6

% Query the current safety status
safetyStatusSubscriber = rossubscriber ('/ dobot_magician / safety_status');
pause (2); % allow the subscriber to receive a message
currentSafetyStatus = safetyStatusSubscriber . LatestMessage . Data ;

% CONFIRM STATUS 4 (OPERATING)

%% Homing
[ safetyStatePublisher , safetyStateMsg ] = rospublisher ('/ dobot_magician /target_safety_status ');
safetyStateMsg . Data = 2; % INITIALISING
send ( safetyStatePublisher , safetyStateMsg );

% WAIT TILL STATUS READS 4

%% EStop
[ safetyStatePublisher , safetyStateMsg ] = rospublisher ('/ dobot_magician /target_safety_status ');
safetyStateMsg . Data = 3; % ESTOPPED
send ( safetyStatePublisher , safetyStateMsg );

%% ================== 3.4 Robot state ================== 

% Query the current joint State
jointStateSubscriber = rossubscriber ('/ dobot_magician / joint_states '); 
% create subscriber
pause (2); 
% allow a message to arrive
currentJointState = jointStateSubscriber . LatestMessage . Position ; 
% read joint positions


%% Query the current end-effector pose
endEffectorPoseSubscriber = rossubscriber ('/ dobot_magician /end_effector_poses ');
pause (2);
msg = endEffectorPoseSubscriber . LatestMessage ;

% Position (m)
currentEndEffectorPosition = [msg . Pose . Position .X, ...
                                msg . Pose . Position .Y, ...
                                msg . Pose . Position .Z];

% Orientation as quaternion [w x y z]
currentEndEffectorQuat = [ msg. Pose . Orientation .W, ...
                            msg. Pose . Orientation .X, ...
                            msg. Pose . Orientation .Y, ...
                            msg. Pose . Orientation .Z];

% Euler angles [ roll pitch yaw] ( rad)
[roll , pitch , yaw] = quat2eul ( currentEndEffectorQuat );

%% Set a target joint state
jointTarget = [0, 0.4 , 0.3 , 0]; % Dobot has 4 joints by default

[ targetJointTrajPub , targetJointTrajMsg ] = rospublisher ('/ dobot_magician/ target_joint_states ');
trajectoryPoint = rosmessage (" trajectory_msgs / JointTrajectoryPoint ");
trajectoryPoint . Positions = jointTarget ;
targetJointTrajMsg . Points = trajectoryPoint ;
send ( targetJointTrajPub , targetJointTrajMsg );

%% Set a target end-effector pose
endEffectorPosition = [0.2 , 0, 0.1]; % [x y z] in metres
endEffectorRotation = [0, 0, 0]; % [ roll pitch yaw] in radians
[ targetEndEffectorPub , targetEndEffectorMsg ] = ...
rospublisher ('/ dobot_magician / target_end_effector_pose ');
% Position
targetEndEffectorMsg . Position .X = endEffectorPosition (1);
targetEndEffectorMsg . Position .Y = endEffectorPosition (2);
targetEndEffectorMsg . Position .Z = endEffectorPosition (3);
% Orientation ( quaternion from Euler ) -> [w x y z]
qua = eul2quat ( endEffectorRotation );
targetEndEffectorMsg . Orientation .W = qua (1);
targetEndEffectorMsg . Orientation .X = qua (2);
targetEndEffectorMsg . Orientation .Y = qua (3);
targetEndEffectorMsg . Orientation .Z = qua (4);
send ( targetEndEffectorPub , targetEndEffectorMsg );

%% ================== 3.5 Robot state ================== 

% State Code
% OFF   0
% ON    1

% Read the current tool state
toolStateSubscriber = rossubscriber ('/ dobot_magician / tool_state ');
pause (2); % allow the subscriber to receive a message
currentToolState = toolStateSubscriber . LatestMessage . Data ;

% Set the suction cup (vacuum pump)
[ toolStatePub , toolStateMsg ] = rospublisher ('/ dobot_magician /target_tool_state ');
toolStateMsg . Data = 1; % turn vacuum ON (use 0 to turn OFF)
send ( toolStatePub , toolStateMsg );


