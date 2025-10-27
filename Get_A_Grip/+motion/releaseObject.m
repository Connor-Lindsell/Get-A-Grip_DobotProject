% =========================================================================
% Release Object Motion
% release object at specified position
%  
%  Inputs:
%    blockPosition - A 3x1 vector representing the (x, y, z) position to release the object
% =========================================================================

function realeaseObject(blockPosition)
    [ toolStatePub , toolStateMsg ] = rospublisher ('/dobot_magician/target_tool_state');

    % Deactivate pump and open gripper
    toolStateMsg . Data = [0, 0]; 
    send ( toolStatePub , toolStateMsg );

    goalPosition = blockPosition + [0.0; 0.0; 0.1]; % hover above block
    goalOrientation = [0.0; 0.0; 0.0]; % block aligned with base

    % Move to hover position above block
    move2goal(goalPosition, goalOrientation)

    pause(2);

end