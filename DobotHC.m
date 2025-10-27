
%% ================== Initialise connection ==================
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
safetyStatusSubscriber = rossubscriber ('/dobot_magician/safety_status');
pause (2); % allow the subscriber to receive a message
msg = receive(safetyStatusSubscriber, 3);
currentSafteyStatus = msg.Data;

fprintf("ran");

% CONFIRM STATUS 4 (OPERATING)

%%
fprintf("Homing\n");
[safetyStatePublisher, safetyStateMsg ] = rospublisher ('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2; % INITIALISING
send ( safetyStatePublisher , safetyStateMsg );

% WAIT TILL STATUS READS 4

%% EStop
[safetyStatePublisher, safetyStateMsg ] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 3; % ESTOPPED
send(safetyStatePublisher,safetyStateMsg );

%% ================== 3.4 Robot state ================== 

%% Find current joint State
jointStateSubscriber = rossubscriber ('/dobot_magician/joint_states'); 
% create subscriber
pause (2); 
% allow a message to arrive
currentJointState = jointStateSubscriber.LatestMessage.Position ; 
% read joint positions

fprintf("\n");
fprintf("Joint States = \n")
disp(currentJointState);
fprintf("\n");



%% Find current end-effector pose
endEffectorPoseSubscriber = rossubscriber ('/dobot_magician/end_effector_poses');
pause (2);
msg = endEffectorPoseSubscriber.LatestMessage ;

% Position (m)
currentEndEffectorPosition = [msg.Pose.Position.X, ...
                              msg.Pose.Position.Y, ...
                              msg.Pose.Position.Z];

% Orientation as quaternion [w x y z]
currentEndEffectorQuat = [msg.Pose.Orientation.W, ...
                          msg.Pose.Orientation.X, ...
                          msg.Pose.Orientation.Y, ...
                          msg.Pose.Orientation.Z];

% Euler angles [ roll pitch yaw] ( rad)
rpy = quat2eul(currentEndEffectorQuat);

roll = quat2eul(currentEndEffectorQuat);

fprintf("\n");
fprintf("End Effector Position = ")
disp(currentEndEffectorPosition);
fprintf("\n");
fprintf("End Effector Quaternion = ")
disp(currentEndEffectorQuat);
fprintf("\n");
fprintf("Roll, Pitch, Yaw = ")
disp(rpy)
fprintf("\n");

%% 1st Movement

endEffectorPosition = [0.2, 0.0, 0.0]; % [x y z] in metres
endEffectorRotation = [0, 0, 0]; % [ roll pitch yaw] in radians
[targetEndEffectorPub, targetEndEffectorMsg ] = ...
rospublisher ('/dobot_magician/target_end_effector_pose');

% Position
targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

% Orientation ( quaternion from Euler ) -> [w x y z]
qua = eul2quat ( endEffectorRotation );
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
send (targetEndEffectorPub , targetEndEffectorMsg );

pause(5);

[ toolStatePub , toolStateMsg ] = rospublisher ('/dobot_magician/target_tool_state');

% Open close
toolStateMsg . Data = [1, 0]; % pump ON , gripper OPEN
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [1, 1]; % pump ON , gripper CLOSE
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [0, 0]; % pump OFF , gripper OPEN ( release )
send ( toolStatePub , toolStateMsg );
pause(2);

% 2nd Movement

endEffectorPosition2 = [0.21, 0.0, 0.14]; % [x y z] in metres
endEffectorRotation = [0, 0, 0]; % [ roll pitch yaw] in radians
[targetEndEffectorPub, targetEndEffectorMsg ] = ...
rospublisher ('/dobot_magician/target_end_effector_pose');

% Position
targetEndEffectorMsg.Position.X = endEffectorPosition2(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition2(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition2(3);

% Orientation ( quaternion from Euler ) -> [w x y z]
qua = eul2quat ( endEffectorRotation );
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send (targetEndEffectorPub , targetEndEffectorMsg );

pause(5);

[ toolStatePub , toolStateMsg ] = rospublisher ('/dobot_magician/target_tool_state');

% Open close
toolStateMsg . Data = [1, 0]; % pump ON , gripper OPEN
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [1, 1]; % pump ON , gripper CLOSE
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [0, 0]; % pump OFF , gripper OPEN ( release )
send ( toolStatePub , toolStateMsg );
pause(2);

% 3rd Movement

endEffectorPosition3 = [0.2, 0.2, 0.02]; % [x y z] in metres
endEffectorRotation = [0, 0, 0]; % [ roll pitch yaw] in radians
[targetEndEffectorPub, targetEndEffectorMsg ] = ...
rospublisher ('/dobot_magician/target_end_effector_pose');

% Position
targetEndEffectorMsg.Position.X = endEffectorPosition3(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition3(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition3(3);

% Orientation ( quaternion from Euler ) -> [w x y z]
qua = eul2quat ( endEffectorRotation );
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send (targetEndEffectorPub , targetEndEffectorMsg );

pause(5);

[ toolStatePub , toolStateMsg ] = rospublisher ('/dobot_magician/target_tool_state');

% Open close
toolStateMsg . Data = [1, 0]; % pump ON , gripper OPEN
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [1, 1]; % pump ON , gripper CLOSE
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [0, 0]; % pump OFF , gripper OPEN ( release )
send ( toolStatePub , toolStateMsg );
pause(2);


% 4th Movement

endEffectorPosition4 = [0.175, -0.18, 0.08]; % [x y z] in metres
endEffectorRotation = [0, 0, 0]; % [ roll pitch yaw] in radians
[targetEndEffectorPub, targetEndEffectorMsg ] = ...
rospublisher ('/dobot_magician/target_end_effector_pose');

% Position
targetEndEffectorMsg.Position.X = endEffectorPosition4(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition4(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition4(3);

% Orientation ( quaternion from Euler ) -> [w x y z]
qua = eul2quat ( endEffectorRotation );
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send (targetEndEffectorPub , targetEndEffectorMsg );

pause(5);

[ toolStatePub , toolStateMsg ] = rospublisher ('/dobot_magician/target_tool_state');

% Open close
toolStateMsg . Data = [1, 0]; % pump ON , gripper OPEN
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [1, 1]; % pump ON , gripper CLOSE
send ( toolStatePub , toolStateMsg );
pause(2);
toolStateMsg . Data = [0, 0]; % pump OFF , gripper OPEN ( release )
send ( toolStatePub , toolStateMsg );
pause(2);
