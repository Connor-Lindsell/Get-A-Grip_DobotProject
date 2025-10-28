% =========================================================================
% MOVE 2 GOAL: Move the robot end-effector to a specified goal pose
% 
% This function computes and executes the necessary motion to move the robot's
% end-effector to a desired goal position and orientation in 3D space.
% 
% Inputs:
%   goalPosition - A 3x1 vector representing the target (x, y, z) position of the end-effector.
%   goalOrientation - A 3x1 vector representing the target (roll, pitch, yaw) orientation of the end-effector in radians.
%  
% =========================================================================

function move2goal(goalPosition, goalOrientation)

    disp('=======================================');
    disp('Moving to goal position:');
    disp(goalPosition);
    disp('With goal orientation (roll, pitch, yaw):');
    disp(goalOrientation);

    endEffectorPosition = goalPosition;
    endEffectorRotation = goalOrientation;
    [targetEndEffectorPub, targetEndEffectorMsg ] = rospublisher ('/dobot_magician/target_end_effector_pose');

    % Position
    targetEndEffectorMsg.Position.X = endEffectorPosition(1);
    targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
    targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

    qua = eul2quat ( endEffectorRotation );
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);
    send (targetEndEffectorPub , targetEndEffectorMsg );
    
    pause(2.5); % wait for motion to complete

    % Commented out while testing
    % % Confirm reaching goal, if goal end effector pose is close enough to goal pose
    % while(!isEndEffectorAtGoal(goalPosition, goalOrientation))
    %     pause(0.5);
    % end

    disp('Reached goal position.');
    disp('=======================================');

end