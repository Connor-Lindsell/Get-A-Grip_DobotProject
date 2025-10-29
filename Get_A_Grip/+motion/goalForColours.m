% =========================================================================
% goalForColours.m
% This function returns the goal position and orientation for a given block
% colour.
% ========================================================================= 

function [goalPosition, goalOrientation] = goalForColours(blockColour)

    switch blockColour
        case 'red'
            goalPosition = [0.1128; -0.1187; -0.0521];
            goalOrientation = [0 0 0];
        case 'green'
            goalPosition = [0.1539; -0.1265; -0.0519];
            goalOrientation = [0 0 0];
        case 'blue'
            goalPosition = [0.1996; -0.1325; -0.0524];
            goalOrientation = [0 0 0];
        otherwise
            error('Unknown block colour: %s', blockColour);
    end

end