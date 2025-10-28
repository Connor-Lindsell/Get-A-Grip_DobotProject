% =========================================================================
% Grab Object Motion
% grab object at specified position
%
%  Inputs:
%    blockPosition - A 3x1 vector representing the (x, y, z) position to grab the object
% =========================================================================

function grabObject(blockPosition,blockOrientation)
    [ toolStatePub , toolStateMsg ] = rospublisher ('/dobot_magician/target_tool_state');

    % Move to hover position above block
    move2goal(blockPosition, blockOrientation)

    % Activate pump and open gripper
    toolStateMsg . Data = [1, 0]; 
    send ( toolStatePub , toolStateMsg );
    pause(2); % wait for gripper to open

    goalPosition = blockPosition + [0.0; 0.0; 0.05]; % hover above block
    goalOrientation = [0.0; 0.0; 0.0]; % block aligned with base

    % Move to hover position above block
    move2goal(goalPosition, goalOrientation)

    pause(2);

    % Activate pump and close gripper to grab block
    toolStateMsg . Data = [1, 1]; % pump ON , gripper CLOSE
    send ( toolStatePub , toolStateMsg );
    pause(2); % wait for gripper to close
end