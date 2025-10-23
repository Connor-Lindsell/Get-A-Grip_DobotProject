%% demo_pick_place_sim.m
% Example usage of the Dobot simulation stack.  The script constructs the
% kinematic model, defines two waypoints, plans the motion, and animates the
% robot in MATLAB using the Peter Corke Robotics Toolbox.

%% Housekeeping
clc; clear; close all;

%% Build simulation helper
sim = DobotSimRunner();
sim.build();

%% Define pick and place targets (metres and radians)
A_xyz = [0.18, 0.00, 0.06]; A_yaw = 0.0;
B_xyz = [0.18, 0.10, 0.06]; B_yaw = 0.0;

%% Generate waypoint sequence with approach and retreat poses
seq = sim.setWaypoints(A_xyz, A_yaw, B_xyz, B_yaw);

%% Choose path type:
%   false -> joint space jtraj between waypoints
%   true  -> Cartesian ctraj and per-step IK (slower, more precise)
useCartesian = false;

%% Run the pick and place simulation
result = sim.runPickPlace(seq, useCartesian);

disp(result)
