% =========================================================================
% isEndEffectorAtGoal.m: Check if the robot end-effector is at the specified goal pose
%
% This function compares the current end-effector position and orientation
% with the desired goal position and orientation to determine if the end-effector
% has reached the goal within specified tolerances.
% Inputs:
%   goalPosition - A 3x1 vector representing the target (x, y, z) position of the end-effector.
%   goalOrientation - A 3x1 vector representing the target (roll, pitch, yaw) orientation of the end-effector in radians.       
% =========================================================================

function atGoal = isEndEffectorAtGoal(goalPosition, goalOrientation)

    disp('=======================================');
    disp('Checking if end-effector is at goal...');

    % Position (m)
    currentEndEffectorPosition = [msg.Pose.Position .X, ...
                                msg.Pose.Position .Y, ...
                                msg.Pose.Position .Z];

    disp('Current End-Effector Position:');
    disp(currentEndEffectorPosition);
    disp('Goal Position:');
    disp(goalPosition');

    % Orientation as quaternion [w x y z]
    currentEndEffectorQuat = [msg.Pose.Orientation .W, ...
                            msg.Pose.Orientation .X, ...
                            msg.Pose.Orientation .Y, ...
                            msg.Pose.Orientation .Z];

    % Euler angles [ roll pitch yaw] ( rad)
    [roll, pitch, yaw] = quat2eul(currentEndEffectorQuat);

    % Current orientation
    currentEndEffectorOrientation = [roll, pitch, yaw];

    disp('Current End-Effector Orientation (roll, pitch, yaw):');
    disp(currentEndEffectorOrientation);
    disp('Goal Orientation (roll, pitch, yaw):');
    disp(goalOrientation');

    % Tolerances
    positionTolerance = 0.01; % meters
    orientationTolerance = 0.05; % radians

    % Check if within tolerances
    positionError = norm(currentEndEffectorPosition - goalPosition');
    orientationError = norm(currentEndEffectorOrientation - goalOrientation');

    % Determine if at goal
    if positionError <= positionTolerance && orientationError <= orientationTolerance
        disp('End-effector is at the goal position and orientation.');
        disp('=======================================');
        atGoal = true;
    else
        disp('End-effector is NOT at the goal position and orientation.');
        disp('=======================================');
        atGoal = false;
    end

end