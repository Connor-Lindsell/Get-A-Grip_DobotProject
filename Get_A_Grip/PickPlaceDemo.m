% =========================================================================
% PickPlaceDemo.m
% in this demo, the robot arm picks up a block from a table and places it
% at a specified location depending on the colour of the block  
% 
% each block will have a block position (where it is picked from), a
% hover position (block position + height offset) to avoid collisions when
% moving and a block orientation (roll, pitch, yaw) for picking up the
% block
% =========================================================================

function PickPlaceDemo()

    % Initialize ROS node
    rosinit ('10.42.0.1'); % IP address of ROS master

    import motion.*
    import perception.*

    %% Properties

    % Hardcode Pose
    block1pos = [0.2; -0.1; 0.05];
    block2pos = [0.2; 0.0; 0.05];
    block3pos = [0.2; 0.1; 0.05];

    % Camera Pose
    % block1pos = getBlockPose3D;
    % block2pos = getBlockPose3D;
    % block3pos = getBlockPose3D;

    ho = 0.05; % height offset for hover position

    %% Properties of blocks
    block1 = struct('colour', 'red', ...
                    'blockPosition', block1pos, ...
                    'hoverPosition', [block1pos(1); block1pos(2); block1pos(3) + ho], ...
                    'blockOrientation', [0; 0; 0]);

    block2 = struct('colour', 'green', ...
                    'blockPosition', block2pos, ...
                    'hoverPosition', [block2pos(1); block2pos(2); block2pos(3) + ho], ...
                    'blockOrientation', [0; 0; 0]);

    block3 = struct('colour', 'blue', ...
                    'blockPosition', block3pos, ...
                    'hoverPosition', [block3pos(1); block3pos(2); block3pos(3) + ho], ...
                    'blockOrientation', [0; 0; 0]);                

    %% Homing
    disp('=======================================');
    disp("Homing\n");
    [safetyStatePublisher, safetyStateMsg ] = rospublisher ('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2; % INITIALISING
    send ( safetyStatePublisher , safetyStateMsg );

    pause(5); % wait for homing to complete
    disp('Homing complete.');

    disp("Openning gripper\n");
    [ toolStatePub , toolStateMsg ] = rospublisher ('/dobot_magician/target_tool_state');

    % Deactivate pump and open gripper
    toolStateMsg . Data = [0, 0]; 
    send ( toolStatePub , toolStateMsg );

    pause(2); % wait for gripper to open
    disp('Gripper opened.');
    disp('READY TO PICK AND PLACE');
    disp('=======================================');

    %% Pick and place block 1

    disp('=======================================');
    disp('Picking and placing block 1:');
    disp('Colour: ');
    disp(block1.colour);
    disp('pick up position: ');
    disp(block1.blockPosition);
    disp('orientation (roll, pitch, yaw): ');
    disp(block1.blockOrientation);
    disp('---------------------------------------');

    move2goal(block1.hoverPosition, block1.blockOrientation); % Move above block

    grabObject(); % Close gripper

    [goalPosition, goalOrientation] = goalForColours(block1.colour);
    move2goal(goalPosition, goalOrientation); % Move to place position

    releaseObject(goalPosition); % Open gripper

    move2goal([0.1; 0; 0.2], [0; 0; 0]); % Move to safe position
    
    disp('Block 1 placed successfully.');
    disp('=======================================');

    %% Pick and place block 2

    disp('=======================================');
    disp('Picking and placing block 2:');
    disp('Colour: ');
    disp(block2.colour);
    disp('pick up position: ');
    disp(block2.blockPosition);
    disp('orientation (roll, pitch, yaw): ');
    disp(block2.blockOrientation);
    disp('---------------------------------------');

    move2goal(block2.hoverPosition, block2.blockOrientation); % Move above block

    grabObject(); % Close gripper

    [goalPosition, goalOrientation] = goalForColours(block2.colour);
    move2goal(goalPosition, goalOrientation); % Move to place position

    releaseObject(goalPosition); % Open gripper

    move2goal([0.1; 0; 0.2], [0; 0; 0]); % Move to safe position

    disp('Block 2 placed successfully.');
    disp('=======================================');

    %% Pick and place block 3

    disp('=======================================');
    disp('Picking and placing block 3:');
    disp('Colour: ');
    disp(block3.colour);
    disp('pick up position: ');
    disp(block3.blockPosition);
    disp('orientation (roll, pitch, yaw): ');
    disp(block3.blockOrientation);
    disp('---------------------------------------');

    move2goal(block3.hoverPosition, block3.blockOrientation); % Move above block

    grabObject(); % Close gripper

    [goalPosition, goalOrientation] = goalForColours(block3.colour);
    move2goal(goalPosition, goalOrientation); % Move to place position

    releaseObject(goalPosition); % Open gripper

    move2goal([0.1; 0; 0.2], [0; 0; 0]); % Move to safe position

    disp('Block 3 placed successfully.');
    disp('=======================================');

    %% Shutdown
    disp('=======================================');
    disp('Pick and place demo complete.');
    disp('Shutting down ROS node...');

    % Close ROS node
    rosshutdown;

    disp('=======================================');

end