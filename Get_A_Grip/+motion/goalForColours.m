% =========================================================================
% goalForColours.m
% This function returns the goal position and orientation for a given block
% colour.
% ========================================================================= 

function [goalPosition, goalOrientation] = goalForColours(blockColour)

    switch blockColour
        case 'red'
            goalPosition = [0.2; -0.1; 0.05];
            goalOrientation = [0; 0; 0];
        case 'green'
            goalPosition = [0.2; 0.0; 0.05];
            goalOrientation = [0; 0; 0];
        case 'blue'
            goalPosition = [0.2; 0.1; 0.05];
            goalOrientation = [0; 0; 0];
        otherwise
            error('Unknown block colour: %s', blockColour);
    end

end